import numpy as np
import cv2
from skimage import color, morphology
from skimage.segmentation import flood
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform


K_3D = 0.9
PENCIL_TOLERANCE = 0.016
WOUND_TOLERANCE = 0.016
SHOW = False

def degree_to_radian(grado):
    return (np.pi/180) * grado

def radian_to_degree(radian):
    return (180/np.pi) * radian

def Mrot(theta):

    C = np.cos(theta)
    S = np.sin(theta)
    R = np.array([[C, -S], [S, C]])
    return R

def flip_y(pose, Image_Y):
    return np.array([pose[0], Image_Y - pose[1]])

def distance(pose1, pose2):
    return np.linalg.norm(pose1 - pose2)

def Jacobian_inv(info: dict):

    q1 = info["q1"]
    q2 = info["q2"]
    L1 = info["L1"]
    L2 = info["L2"]

    a = - (L1 * np.sin(q1) + L2 * np.sin(q1 + q2))
    b = - (L2 * np.sin(q1 + q2))
    c =  L1 * np.cos(q1) + L2 * np.cos(q1 + q2) 
    d = L2 * np.cos(q1 + q2)

    J = np.array([[a, b], [c, d]])
    Jinv = np.linalg.pinv(J)
    return Jinv

def inverse_kinematics(dimension: int, info: dict):

    if dimension == 2:
        return inverse_kinematics_2D(info)
    elif dimension == 3:
        return inverse_kinematics_3D(info)
    else:
        return None
    
def inverse_kinematics_2D(info: dict):

    pose_deseada = np.array([info["pose_deseada"][0], info["pose_deseada"][1]])
    actual_q1 = info["q1"]
    actual_q2 = info["q2"]
    precision = info["precision"]
    max_steps = info["max_steps"]

    #Calcula la cinematica inversa del brazo robotico
    #pose_deseada: Posicion deseada del extremo del brazo
    actual_pose = pose_calc(2, info)
    Error = pose_deseada - actual_pose
    dist = distance(actual_pose, pose_deseada)

    New_q1 = actual_q1
    New_q2 = actual_q2
    count = 0
    pos_info = {"q1": New_q1, "q2": New_q2, "L1": info["L1"], "L2": info["L2"]}
    while dist > precision and count < max_steps:

        J = Jacobian_inv(New_q1, New_q2)
        correction = np.dot(J, Error)
        New_q1 += correction[0]
        New_q2 += correction[1]

        pos_info["q1"] = New_q1
        pos_info["q2"] = New_q2
        
        Error = pose_deseada - pose_calc(2, pos_info)
        dist = distance(pose_calc(2, pos_info), pose_deseada)
        count += 1

    if dist > precision:
        return np.array([None, None])

    return np.array([New_q1, New_q2])

def inverse_kinematics_3D(info: dict):

    pos_2d = inverse_kinematics_2D(info)
    New_q1 = pos_2d[0]
    New_q2 = pos_2d[1]
    pose_deseada = info["pose_deseada"]
    actual_z = info["z"]
    precision = info["precision"]
    max_steps = info["max_steps"]
    
    actual_pose = pose_calc(3, info)
    Error = pose_deseada - actual_pose
    dist = distance(actual_pose, pose_deseada)
    New_z = actual_z
    
    count = 0
    pos_info = {"q1": New_q1, "q2": New_q2, "L1": info["L1"], "L2": info["L2"], "z": New_z}
    while dist > precision and count < max_steps:

        correction3D = Error[2] * K_3D
        New_z += correction3D

        J = Jacobian_inv(New_q1, New_q2)
        correction2D = np.dot(J, Error)
        New_q1 += correction2D[0]
        New_q2 += correction2D[1]

        pos_info["z"] = New_z
        pos_info["q1"] = New_q1
        pos_info["q2"] = New_q2

        Error = pose_deseada - pose_calc(3, pos_info)
        dist = distance(pose_calc(3, pos_info), pose_deseada)
        count += 1
    
    if dist > precision:
        return np.array([None, None])
        
    return np.array([New_q1, New_q2, New_z])

def pose_calc(dimension: int, info: dict):

    if dimension == 2:
        return pose_calc_2D(info)
    elif dimension == 3:
        return pose_calc_3D(info)
    else:
        return None
    
def pose_calc_2D(info: dict):

    q1 = info["q1"]
    q2 = info["q2"]
    L1 = info["L1"]
    L2 = info["L2"]
    pencil_diff = info["pencil_diff"] #Diferencia entre la punta del lapiz y el extremo del brazo horizontalmente

    #Calcula una posicion en x e y del extremo efector segun un q1 y q2 dados
    x = L1 * np.cos(q1) + (L2 - pencil_diff) * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + (L2 - pencil_diff) * np.sin(q1 + q2)
    return np.array([x, y])

def pose_calc_3D(info: dict):

    height = info["height"]
    pencil_height = info["pencil_height"] #Diferencia entre la punta del lapiz y el extremo del brazo verticalmente

    #Calcula una posicion del extremo efector segun q1, q2 y altura dados
    pos_xy = pose_calc_2D(info)
    x = pos_xy[0]
    y = pos_xy[1]
    z = height - pencil_height
    return np.array([x, y, z])

def find_wound(image):
    #Calcula la posicion del objetivo en la imagen en pixeles
    
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
    red_mask = cv2.add(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)  
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea)

    M = cv2.moments(largest_contour)
    if M["m00"] != 0:  # Para evitar división por cero
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0

    point_center = np.array([cx, cy])
    return point_center, largest_contour


def find_pencil(image, info: dict):
    #Calcula la posicion del extremo efector en la imagen en pixeles

    img_hsv = color.rgb2hsv(image)
    img_hsv_copy = np.copy(img_hsv)
    init_pose = info["center"] - np.array([0, info["MAX_Y"]//4]) #Toma el centro de la imagen y le resta un cuarto de la altura
    init_pose = list(init_pose)

    mask = flood(img_hsv[..., 2], init_pose, tolerance= PENCIL_TOLERANCE)
    img_hsv[mask, 2] = 1

    mask_postprocessed = morphology.binary_opening(mask, np.ones((3, 3)))
    mask_postprocessed = morphology.binary_closing(mask_postprocessed, morphology.disk(20))
    img_hsv_copy[mask_postprocessed, 2] = 1

    mask_post_array = np.array(mask_postprocessed)
    mask_post_x, mask_post_y = np.nonzero(mask_post_array)

    max_y = np.max(mask_post_y)
    max_y_index = np.argmax(mask_post_y)
    max_x = mask_post_x[max_y_index]

    if SHOW:
        plt.imshow(color.hsv2rgb(img_hsv))
        plt.show() #Muestra la imagen con el flood fill sin postprocesar

        plt.imshow(color.hsv2rgb(img_hsv_copy))
        plt.show() #Muestra la imagen con el flood fill postprocesado

        plt.imshow(color.hsv2rgb(img_hsv_copy))
        plt.plot(max_x, max_y, "og", markersize=3) #Coloca un punto verde en el punto encontrado
        plt.show() #Muestra la imagen con el flood fill postprocesado y el punto encontrado

    return np.array([max_x, max_y])

def define_stitching_points(image, info: dict):
    pix_to_mm = info["pix_to_mm"]
    
    wound_center_pixel, wound_contour_pixel = find_wound(image)

    wound_countour_mm = wound_contour_pixel - wound_center_pixel #Centro el contorno en el origen
    wound_countour_mm = wound_countour_mm * pix_to_mm #Convierto de pixeles a milimetros
    
    distance_matrix = squareform(pdist(wound_countour_mm))
    _ , wound_length_mm = solve_nearest_neighbor({"distance_matrix": distance_matrix})

    


    


def translate_wound_geometry(image, info: dict):

    sift = info["sift"]
    flann = info["flann"]
    wound_descriptor = info["wound_descriptor"]
    minimum_matches = info["minimum_matches"]
    wound_keypoints = info["wound_keypoints"]
    initial_stitches = info["initial_stitches"]
    target = info["target"]

    found_keypoints, found_descriptors = sift.detectAndCompute(image, None)
    matches = flann.knnMatch(found_descriptors, wound_descriptor, k=2)
    good_matches = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good_matches.append(m)
    
    if len(good_matches) >= minimum_matches:
        source_pts = np.float32([ found_keypoints[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
        destination_pts = np.float32([ wound_keypoints[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(source_pts, destination_pts, cv2.RANSAC,5.0)

        init_point = np.array([initial_stitches[target]])
        new_pixel = cv2.perspectiveTransform(init_point, M)
        new_pixel = np.int32(new_pixel[0])     

        return new_pixel
    
    else:
        return None

def order_stitches(info: dict):
    #Ordena los puntos de sutura para moverse optimamente
    stitches = info["stitches"]
    distance_matrix = squareform(pdist(stitches))
    info["distance_matrix"] = distance_matrix
    path, total_distance = solve_nearest_neighbor(info)
    return path, total_distance

def solve_nearest_neighbor(info: dict):
    #Algoritmo del vecino más cercano

    distance_matrix = info["distance_matrix"]
    if "start" in info.keys():
        start = info["start"]
    else:
        start = 0

    n = len(distance_matrix)
    visited = [False] * n
    path = [start]
    total_distance = 0

    current = start
    visited[current] = True

    for _ in range(n - 1):
        next_point = np.argmin(
            [distance_matrix[current][j] if not visited[j] else float('inf') for j in range(n)])
        total_distance += distance_matrix[current][next_point]
        path.append(next_point)
        visited[next_point] = True
        current = next_point

    return path, total_distance