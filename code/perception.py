import numpy as np
import cv2
import math

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Threshold for rock: R > thresh_R & G > thresh_G & B < thresh_B
def rock_thresh(img, rgb_thresh=(140, 110, 90)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])

    match_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[match_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    dst_size = 5
    bottom_offset = 6
    img = Rover.img

    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dst = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    
    warped = perspect_transform(img, src, dst)

    rock_img = rock_thresh(warped)
    nav_img = color_thresh(warped)

    # Thresh of rock sample
    Rover.vision_image[:, :, 1] = rock_img * 255
    # Thresh of navigable
    Rover.vision_image[:, :, 2] = nav_img * 255

    one_img = np.ones_like(img[:, :, 0])
    warped_one = perspect_transform(one_img, src, dst)
    
    ob_img = np.zeros_like(img[:, :, 0])
    ob_img[(warped_one == 1) & (nav_img == 0) & (rock_img == 0)] = 1
    # Thresh of obstacle
    Rover.vision_image[:, :, 0] = ob_img * 255

    pitch = 360. - Rover.pitch if Rover.pitch > 180. else math.fabs(Rover.pitch)
    roll = 360. - Rover.roll if Rover.roll > 180. else math.fabs(Rover.roll)

    thresh_angle = 0.5

    if pitch <= thresh_angle and roll <= thresh_angle:
        # Map obstacles
        xpix, ypix = rover_coords(ob_img)
        xpix_world, ypix_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
        Rover.worldmap[ypix_world, xpix_world, 0] += 1
        # Map navigable terrian
        xpix, ypix = rover_coords(rock_img)
        rock_dists, rock_angles = to_polar_coords(xpix, ypix)
        xpix_world, ypix_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
        Rover.worldmap[ypix_world, xpix_world, 1] += 1
        
        # Map navigable terrian
        xpix, ypix = rover_coords(nav_img)
        xpix_world, ypix_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
        Rover.worldmap[ypix_world, xpix_world, 2] += 1

        # Find rock for pickup
        if Rover.picking_up:
            Rover.rock_angle = None
            Rover.rock_yaw = None

        xpix, ypix = rover_coords(nav_img[110:, :])
        Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)

        if len( rock_angles ) > 10:
            a = np.mean( rock_angles ) * 180 / np.pi
            d = np.mean( rock_dists )
            nav_ds, nav_as = to_polar_coords(xpix, ypix)
            nav_as = nav_as * 180 / np.pi
            # if ( a > 45 and d < 50 ) or ( a < 10 and d < 20 ):
            
            print( '===> rock: {:>4d}, dist = {:.1f}, angle = {:.1f}, count = {}'.format( \
                len( rock_angles ), np.mean( rock_dists ), \
                np.mean( rock_angles ) * 180 / np.pi, \
                np.count_nonzero( np.fabs( nav_as - a ) < 10 ) ) )

            if d < 80 and np.count_nonzero( np.fabs( nav_as - a ) < 10 ) > 50:
                Rover.rock_angle = a
                Rover.rock_yaw = ( Rover.yaw + a + 360 ) % 360

    else:
        xpix, ypix = rover_coords(nav_img[110:, :])
        Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)
    
    return Rover