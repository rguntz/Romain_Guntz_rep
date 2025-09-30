import numpy as np

from scipy import signal #for the scipy.signal.convolve2d function
from scipy import ndimage #for the scipy.ndimage.maximum_filter
from scipy.signal import convolve2d
import cv2
from scipy.ndimage import maximum_filter



# Harris corner detector
def extract_harris(img, sigma = 1.0, k = 0.05, thresh = 1e-5):
    '''
    Inputs:
    - img:      (h, w) gray-scaled image
    - sigma:    smoothing Gaussian sigma. suggested values: 0.5, 1.0, 2.0
    - k:        Harris response function constant. suggest interval: (0.04 - 0.06) -> controlling sensitivity to corners
    - thresh:   scalar value to threshold corner strength. suggested interval: (1e-6 - 1e-4)
    Returns:
    - corners:  (q, 2) numpy array storing the keypoint positions [x, y]
    - C:     (h, w) numpy array storing the corner strength
    '''
    # Convert to float
    img = img.astype(float) / 255.0

    # 1. Compute image gradients in x and y direction
    # TODO: implement the computation of the image gradients Ix and Iy here.
    # You may refer to scipy.signal.convolve2d for the convolution.
    # Do not forget to use the mode "same" to keep the image size unchanged.
    print("Hello World!")

    # kernels Sobel 
    sobel_x = np.array([[-1, 0, 1],
                        [-2, 0, 2],
                        [-1, 0, 1]])

    sobel_y = np.array([[-1, -2, -1],
                        [ 0,  0,  0],
                        [ 1,  2,  1]])

    # Convolve the image with the Sobel kernels
    Ix = convolve2d(img, sobel_x, mode='same')  # Gradient in the x direction with "same" configuration
    Iy = convolve2d(img, sobel_y, mode='same')  # Gradient in the y direction with "same" configuration


    #raise NotImplementedError
    
    # 2. (Optional) Blur the computed gradients
    # TODO: compute the blurred image gradients
    # You may refer to cv2.GaussianBlur for the gaussian filtering (border_type=cv2.BORDER_REPLICATE)
    # Apply Gaussian blur to the gradients
    Ix_blur = cv2.GaussianBlur(Ix, (3, 3), sigmaX=sigma, borderType=cv2.BORDER_REPLICATE)
    Iy_blur = cv2.GaussianBlur(Iy, (3, 3), sigmaX=sigma, borderType=cv2.BORDER_REPLICATE)


    


    # 3. Compute elements of the local auto-correlation matrix "M"
    # TODO: compute the auto-correlation matrix here
    # You may refer to cv2.GaussianBlur or scipy.signal.convolve2d to perform the weighted sum
    Ix_squared = Ix_blur ** 2
    Iy_squared = Iy_blur ** 2
    Ixy = Ix_blur * Iy_blur

    # 4. Compute the elements of the local auto-correlation matrix M
    M11 = cv2.GaussianBlur(Ix_squared, (3, 3), sigmaX=sigma, borderType=cv2.BORDER_REPLICATE)  
    M22 = cv2.GaussianBlur(Iy_squared, (3, 3), sigmaX=sigma, borderType=cv2.BORDER_REPLICATE)  
    M12 = cv2.GaussianBlur(Ixy, (3, 3), sigmaX=sigma, borderType=cv2.BORDER_REPLICATE)      





    # 4. Compute Harris response function C
    # TODO: compute the Harris response function C here
    det_M = M11 * M22 - M12**2
    trace_M = M11 + M22
    C = det_M - k*trace_M



    

    # 5. Detection with threshold and non-maximum suppression
    # TODO: detection and find the corners here
    # For the non-maximum suppression, you may refer to scipy.ndimage.maximum_filter to check a 3x3 neighborhood.
    # You may refer to np.where to find coordinates of points that fulfill some condition; Please, pay attention to the order of the coordinates.
    # You may refer to np.stack to stack the coordinates to the correct output format


    strong_corners = C > thresh  # Create a boolean mask for strong corners
    # Apply maximum filter for non-maximum suppression
    C_nms = maximum_filter(C, size=3)  # Perform maximum filtering with a 3x3 neighborhood

    # Create a mask for local maxima
    local_max = (C == C_nms)  # Keep only local maxima

    # Combine local maxima with the strong corners threshold
    corners_mask = local_max & strong_corners  # Only keep corners that are both local maxima and above the threshold

    # Extract coordinates of corners
    y_coords, x_coords = np.where(corners_mask)  # Get coordinates where corners are True

    # Stack coordinates
    corners = np.stack((x_coords, y_coords), axis=-1)  # Stack the coordinates for output

        



    return corners, C