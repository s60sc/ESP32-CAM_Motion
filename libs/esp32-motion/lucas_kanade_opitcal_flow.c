#include "motion.h"
#include "convolution.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

static const float NoiseThreshold = 0.01;// Lucas Kanade noise threshold
static const int half_window = WINDOW / 2;
static const int window_squared = WINDOW * WINDOW;
static const int log2_window = (int)ceil(log2(WINDOW));

// define 5x5 Gaussian kernel flattened
static const float kernel[WINDOW * WINDOW] = {1 / 256.0f, 4 / 256.0f, 6 / 256.0f, 4 / 256.0f, 1 / 256.0f, 4 / 256.0f, 16 / 256.0f,
	24 / 256.0f, 16 / 256.0f, 4 / 256.0f, 6 / 256.0f, 24 / 256.0f, 36 / 256.0f, 24 / 256.0f, 6 / 256.0f, 4 / 256.0f,
	16 / 256.0f, 24 / 256.0f, 16 / 256.0f, 4 / 256.0f, 1 / 256.0f, 4 / 256.0f, 6 / 256.0f, 4 / 256.0f, 1 / 256.0f};

// Separable Gaussian kernel
static const float Kernel_isotropic[WINDOW] = {1.0 / 16.0, 4.0 / 16.0, 6.0 / 16.0, 4.0 / 16.0, 1.0 / 16.0 };

// Differentiate Lucas Kanade kernel https://www.cs.toronto.edu/~fleet/research/Papers/ijcv-94.pdf
static const float Kernel_Dxy[WINDOW] = {-1.0 / 12.0, 8.0 / 12, 0, -8.0 / 12.0, 1.0 / 12.0};

/// Optical flow Lucas-Kanade
/** @brief Implement LK optical flow source from wiki and matlab:
 * https://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method
 * @param src1 pointer to grayscale buffer image instant t. 
 * @param src2 pointer to grayscale buffer image instant t+1.
 * @param V [out] vector (vx, vy) and squared magnitude
 * @return True if success False if failed somewhere*/
bool LK_optical_flow(const uint8_t *src1, const uint8_t *src2, MotionVector16_t *V, int w, int h, int *mag_max2) {

	const int N = w * h;
	MotionVector16_t *mv = V;
	float *image1 = (float*)malloc(N * sizeof(float)),
		*image2 = (float*)malloc(N * sizeof(float)),
		*fx = (float*)malloc(N * sizeof(float)),
		*ft = (float*)malloc(N * sizeof(float)),
		*fy = (float*)malloc(N * sizeof(float));
	int i, j, m;
	*mag_max2 = 0;

	if(!fx || !fy || !ft || !image1 || !image2) 
		return false;
	else if(!N) 
		return false;
	else if(!src1 || !src2) 
		return false;

	// init input
	for(i = N; i--; ) {
		fx[i] = src1[i];
		ft[i] = src2[i];
		mv->mag2 = 0;
		mv->vx = 0;
		mv->vy = 0;
		mv++;
	}	
	mv = &V[0];

	// Gradient computation: I_{t+1} - I_{t} 
	// and fy initialisation as smoothed input = fx
	for(i = N; --i; ) {
		const float x = fx[i];
		ft[i] -= x;
		fy[i] = x;
	}

	// Derivate Dx : 1D convolution horizontal
	if(!convH(fx, image1, w, h, Kernel_Dxy, 5)) 
		return false;
	
	// Derivate Dy : 1D convolution vertical
	if(!convV(fy, image2, w, h, Kernel_Dxy, 5)) 
		return false;

	// ##Isotropic smooth
	if(!convolve2DSeparable(image1, fx, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) 
		return false;

	if(!convolve2DSeparable(image2, fy, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5))
		return false;

	if(!convolve2DSeparable(ft, image2, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) 
		return false;
	
	memcpy(ft, image2, N * sizeof(float));

	// Lucas Kanade optical flow algorithm
	for(i = half_window; i < h - half_window; ++i) {
		for(j = half_window; j < w - half_window; ++j) {
			float Atb0 = 0, Atb1 = 0; 
			float a = 0, b = 0, c = 0;
			for(m = 0; m < window_squared; ++m) {
				// Sum over the window W²
				const float W = kernel[m];
				const int i_window = (m >> log2_window) - half_window;
				const int j_window = (m % WINDOW) - half_window;
                const unsigned index = (j + j_window) + (i + i_window) * w;
				const float Ix = (float) fx[index] * W;
				const float Iy = (float) fy[index] * W;
				const float It = (float) ft[index] * W;
				a += Ix * Ix;
				c += Iy * Iy; 
				b += Ix * Iy;
				Atb0 += - Ix * It;
				Atb1 += - Iy * It;
			}
			
			//const float eigenval1 = ((a + c) + sqrtf(4 * b * b + powf(a - c, 2))) /2;
			//const float eigenval2 = ((a + c) - hypotf(2 * b, a - c)) * 0.5;

			if(((a + c) - hypotf(2 * b, a - c)) * 0.5 >= NoiseThreshold) {
				//Case 1: λ1≥λ2≥τ equivalent to λ2≥τ
				//A is nonsingular, the system of equations are solved using Cramer's rule.
				const float det = a * c - b * b;
				const float b_by_det = b / det;
				const float iAtA[2][2] = {{  c / det, - b_by_det},
										  {- b_by_det,   a / det}};
				//optical flow : [Vx Vy] = inv[AtA] . Atb
				const float vx = iAtA[0][0] * Atb0 + iAtA[0][1] * Atb1;
				const float vy = iAtA[1][0] * Atb0 + iAtA[1][1] * Atb1;	
				mv = &V[i * w + j];
				mv->vx = (int16_t)vx;
				mv->vy = (int16_t)vy;
				mv->mag2 = (uint16_t)(vx * vx + vy * vy);	
				if(*mag_max2 < mv->mag2)
					*mag_max2 = mv->mag2;		
			} 
		}
    }

	free(fx); free(image1);
	free(fy); free(image2);
	free(ft);

	return true;
}


/** @brief Implement LK optical flow 8bit version Magnitude
 * @param src1 pointer to grayscale buffer image instant t. 
 * @param src2 pointer to grayscale buffer image instant t+1.
 * @param out [out] image 8bit depth output of squared magnitude
 * @return True if success False if failed somewhere*/
bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *out, int w, int h) {
	const int N = w * h;
	uint16_t *tmpMagArray = (uint16_t*)calloc(N, sizeof(uint16_t));
	uint16_t *pMag = tmpMagArray;
	uint16_t maxMag = 0;
	float *image1 = (float*)malloc(N * sizeof(float)),
		*image2 = (float*)malloc(N * sizeof(float)),
		*fx = (float*)malloc(N * sizeof(float)),
		*ft = (float*)malloc(N * sizeof(float)),
		*fy = (float*)malloc(N * sizeof(float));
	int i, j, m;

	if(!fx || !fy || !ft || !image1 || !image2) {
		return false;
	} else if(!N) {
		return false;
	} else if(!src1 || !src2) {
		return false;
	}

	// init input
	for(i = N; i--; ) {
		fx[i] = src1[i];
		ft[i] = src2[i];
	}	
	// Gradient computation: I_{t+1} - I_{t} 
	// and fy initialisation as smoothed input = fx
	for(i = N; --i; ) {
		const float x = fx[i];
		ft[i] -= x;
		fy[i] = x;
	}

	// Derivate Dx : 1D convolution horizontal
	if(!convH(fx, image1, w, h, Kernel_Dxy, 5)) {
		return false;
	}
	
	// Derivate Dy : 1D convolution vertical
	if(!convV(fy, image2, w, h, Kernel_Dxy, 5)) {
		return false;
	}

	// ##Isotropic smooth
	if(!convolve2DSeparable(image1, fx, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		return false;
	}

	if(!convolve2DSeparable(image2, fy, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		return false;
	}
	
	if(!convolve2DSeparable(ft, image2, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		return false;
	}
	
	memcpy(ft, image2, N * sizeof(float));

	memset(out, 0, N);
	// Lucas Kanade optical flow algorithm
	for(i = half_window; i < h - half_window; ++i) {
		for(j = half_window; j < w - half_window; ++j) {
			float Atb0 = 0, Atb1 = 0; 
			float a = 0, b = 0, c = 0;
			for(m = 0; m < window_squared; ++m) {
				// Sum over the window W²
				const float W = kernel[m];
				const int i_window = m / WINDOW - half_window;
				const int j_window = m % WINDOW - half_window;
                const unsigned index = (j + j_window) + (i + i_window) * w;
				const float Ix = (float) fx[index] * W;
				const float Iy = (float) fy[index] * W;
				const float It = (float) ft[index] * W;
				a += Ix * Ix;
				c += Iy * Iy; 
				b += Ix * Iy;
				Atb0 += - Ix * It;
				Atb1 += - Iy * It;
			}
			
			//const float eigenval1 = ((a + c) + sqrtf(4 * b * b + powf(a - c, 2))) /2;
			const float eigenval2 = ((a + c) - hypotf(2 * b, a - c)) * 0.5;

			if(eigenval2 >= NoiseThreshold) {
				//Case 1: λ1≥λ2≥τ equivalent to λ2≥τ
				//A is nonsingular, the system of equations are solved using Cramer's rule.
				const float det = a * c - b * b;
				const float b_by_det = b / det;
				const float iAtA[2][2] = {{  c / det, - b_by_det},
										  {- b_by_det,   a / det}};
				//optical flow : [Vx Vy] = inv[AtA] . Atb
				const float vx = iAtA[0][0] * Atb0 + iAtA[0][1] * Atb1;
				const float vy = iAtA[1][0] * Atb0 + iAtA[1][1] * Atb1;	
				pMag = &tmpMagArray[i * w + j] ;
				*pMag = (uint16_t)(vx * vx + vy * vy);
				if(*pMag > maxMag)
					maxMag = *pMag;
			}
		}
    }

	for(pMag = &tmpMagArray[0], i = 0; i < N; i++, pMag++)
		out[i] = (uint8_t)(*pMag * 255.0 / (float)maxMag);

	free(fx); free(image1);
	free(fy); free(image2);
	free(ft); free(tmpMagArray);
	return true;
}