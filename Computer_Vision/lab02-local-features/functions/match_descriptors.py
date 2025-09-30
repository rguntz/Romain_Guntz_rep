import numpy as np

def ssd(desc1, desc2):
    '''
    Sum of squared differences
    Inputs:
    - desc1:        - (q1, feature_dim) descriptor for the first image
    - desc2:        - (q2, feature_dim) descriptor for the first image
    Returns:
    - distances:    - (q1, q2) numpy array storing the squared distance
    '''
    assert desc1.shape[1] == desc2.shape[1] ## verify the second dimension 

    ## (i, j) is the sum of squared differences between the i-th descriptor in desc1 and the j-th descriptor in desc2.

    # Compute the squared differences using broadcasting
    # We subtract desc2 from desc1, which will broadcast the shapes correctly
    diff = desc1[:, np.newaxis, :] - desc2[np.newaxis, :, :]  

    # Square the differences and sum along the last axis (descriptor_dim)
    distances = np.sum(diff ** 2, axis=2)  # Shape will be (q1, q2)

    return distances








def match_descriptors(desc1, desc2, method = "one_way", ratio_thresh=0.5):
    '''
    Match descriptors
    Inputs:
    - desc1:        - (q1, feature_dim) descriptor for the first image
    - desc2:        - (q2, feature_dim) descriptor for the first image
    Returns:
    - matches:      - (m x 2) numpy array storing the indices of the matches
    '''
    assert desc1.shape[1] == desc2.shape[1]
    distances = ssd(desc1, desc2)
    q1, q2 = desc1.shape[0], desc2.shape[0]
    matches = None
    if method == "one_way": # Query the nearest neighbor for each keypoint in image 1
        # TODO: implement the one-way nearest neighbor matching here
        # You may refer to np.argmin to find the index of the minimum over any axis
        matches = [(i, np.argmin(row)) for i, row in enumerate(distances)]
        matches = np.array(matches)

        

    elif method == "mutual":
        # TODO: implement the mutual nearest neighbor matching here
        # You may refer to np.min to find the minimum over any axis

        matches_1 = [(i, np.argmin(row)) for i, row in enumerate(distances)]


        min_indices = np.argmin(distances, axis=0)

        # Find the indeces of the minimum according to the columns
        matches_2 = [(i, j) for j, i in enumerate(min_indices)]

        # Common cordinates between the minimum according to rows and the minimum according to columns
        common_coordinates = set(matches_1).intersection(matches_2)
        matches = list(common_coordinates)

        # Convert it to numpy array 
        matches = np.array(matches)



    elif method == "ratio":
        # TODO: implement the ratio test matching here
        # You may use np.partition(distances,2,axis=0)[:,1] to find the second smallest value over a row

        min_coords = []  # Initialize as a list
        second_min_coords = []  # Initialize as a list

        matrix = distances

        for row_idx in range(matrix.shape[0]):
            # Use np.partition to find the first two smallest values
            partitioned_row = np.partition(matrix[row_idx], 2)
            
            # Find the minimum and second minimum values
            min_value = partitioned_row[0]
            second_min_value = partitioned_row[1]
            
            # Get the indices of the minimum and second minimum
            min_index = np.where(matrix[row_idx] == min_value)[0][0]
            second_min_index = np.where(matrix[row_idx] == second_min_value)[0][0]

            # Save the coordinates
            min_coords.append((row_idx, min_index))
            second_min_coords.append((row_idx, second_min_index))

        # Convert the lists to arrays after the loop
        min_coords = np.array(min_coords)
        second_min_coords = np.array(second_min_coords)

        final_coords = []
        for i in range(len(min_coords)):
            min_coord = min_coords[i]
            second_min_coord = second_min_coords[i]
            
            # Extract the actual distance values
            min_distance = distances[min_coord[0], min_coord[1]]  # Using row and column indices
            second_min_distance = distances[second_min_coord[0], second_min_coord[1]]  # Using row and column indices

            # Perform the comparison
            if min_distance < second_min_distance * ratio_thresh: 
                final_coords.append(min_coord)

        matches = np.array(final_coords)  # Convert final_coords to a NumPy array


        
    else:
        raise NotImplementedError


    return matches

