import numpy as np
import cv2
from skimage import color, morphology
from skimage.segmentation import flood
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform


K_3D = 0.9
PENCIL_TOLERANCE = 0.016
WOUND_TOLERANCE = 0.016
SHOW = True
SHOW_TIME = 3000
SAVE = False

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
    pencil_diff = info["pencil_diff"]

    x = pose_deseada[0]
    y = pose_deseada[1]
    L1 = info["L1"]
    L2 = info["L2"] - pencil_diff

    # Calcular el ángulo theta2
    theta2 = np.arccos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
    
    if x < 0 and y < 0:
        theta2 = -theta2
    
    # Calcular el ángulo theta1
    theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    
    # Convertir ángulos de radianes a grados
    theta2_deg = -theta2 * 180 / np.pi
    theta1_deg = theta1 * 180 / np.pi
    
    if y == 0:
        print(f"Punto no valido")
        return None
    # Ajustes de ángulos según el cuadrante
    if x >= 0 and y >= 0:  # 1er cuadrante
        theta1_deg = 90 - theta1_deg
    elif x < 0 and y > 0:  # 2do cuadrante
        theta1_deg = 90 - theta1_deg
    elif x < 0 and y < 0:  # 3er cuadrante
        theta1_deg = -270 - theta1_deg
    elif x > 0 and y < 0:  # 4to cuadrante
        theta1_deg = 90 - theta1_deg
    elif x < 0 and y == 0:
        print(f"Punto no valido")
        return None

    # Redondear ángulos
    theta1_deg = round(theta1_deg,3)
    theta2_deg = round(theta2_deg,3)

    return theta1_deg, theta2_deg

def inverse_kinematics_3D(info: dict):

    pos_2d = inverse_kinematics_2D(info)
    New_q1 = pos_2d[0]
    New_q2 = pos_2d[1]
    pose_deseada = info["pose_deseada"]
    New_z = pose_deseada[2]   
        
    return np.array([New_q1, New_q2, New_z])

def pose_calc(dimension: int, info: dict):

    if dimension == 2:
        return pose_calc_2D(info)
    elif dimension == 3:
        return pose_calc_3D(info)
    else:
        return None
    

    
def pose_calc_2D(info: dict):

    theta1 = info["q1"]
    theta2 = info["q2"]
    L1 = info["L1"]
    L2 = info["L2"]-info["pencil_diff"]
    #Calcula una posicion en x e y del extremo efector segun un q1 y q2 dados

    # Conversión de grados a radianes
    theta1_rad = np.radians(theta1-120)  # Grados a radianes
    theta2_rad = np.radians(theta2-128)

    # Cálculo de las coordenadas xP e yP
    yP = round(L1 * np.cos(theta1_rad) + L2 * np.cos(theta1_rad + theta2_rad))
    xP = round(L1 * np.sin(theta1_rad) + L2 * np.sin(theta1_rad + theta2_rad))

    return round(xP, 2), round(yP, 2)

def pose_calc_3D(info: dict):

    #Calcula una posicion del extremo efector segun q1, q2 y altura dados
    x, y = pose_calc_2D(info)
    z = info["z"]
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
    if len(contours) == 0:
        return None, None

    largest_contour = max(contours, key=cv2.contourArea)

    
    if SHOW:
        cv2.drawContours(image, [largest_contour], 0, (0,255,0), 3)
        cv2.imshow("imagen_contorno",image)
        cv2.waitKey(SHOW_TIME)
        if SAVE:
            save_image(image, "wound_countour")

    M = cv2.moments(largest_contour)
    if M["m00"] != 0:  # Para evitar división por cero
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0

    point_center = np.array([cx, cy])
    print("wound center")
    print(point_center)
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

def find_perpendicular_edge(info: dict):

    wound_countour_mm = info["wound_countour_mm"]
    actual_point = info["actual_point"]
    length_axis = info["length_axis"]
    tolerance = info["tolerance"]
    perp_vec = np.array((-length_axis[1], length_axis[0]))

    projection_wound = wound_countour_mm@length_axis
    projection_actual = np.dot(actual_point, length_axis)

    matching_points_index = np.where(np.abs(projection_wound - projection_actual) <= tolerance)
    matching_wound_points = wound_countour_mm[matching_points_index]
    
    projection_wound_perp = matching_wound_points@perp_vec
    sup_match_index = np.where(projection_wound_perp > 0)
    inf_match_index = np.where(projection_wound_perp < 0)

    if len(sup_match_index[0]) > 0 and len(inf_match_index[0]) > 0:
        sup_matching_points = matching_wound_points[sup_match_index]
        inf_matching_points = matching_wound_points[inf_match_index]

        sup_dist = np.mean(sup_matching_points@perp_vec)
        inf_dist = np.mean(inf_matching_points@perp_vec)

    elif len(sup_match_index[0]) > 0 and len(inf_match_index[0]) == 0:
        sup_matching_points = matching_wound_points[sup_match_index]
        sup_dist = np.mean(sup_matching_points@perp_vec)
        inf_dist = - sup_dist
    
    elif len(sup_match_index[0]) == 0 and len(inf_match_index[0]) > 0:

        inf_matching_points = matching_wound_points[inf_match_index]
        inf_dist = np.mean(inf_matching_points@perp_vec)
        sup_dist = - inf_dist

    else:
        info["tolerance"] = tolerance * 1.05
        sup_dist, inf_dist = find_perpendicular_edge(info)
    

    return sup_dist, inf_dist


def define_stitching_points(image, info: dict):
    pix_to_mm = info["pix_to_mm"]
    wound_center_pixel, wound_contour_pixel = find_wound(image)

    wound_countour_mm = wound_contour_pixel - wound_center_pixel #Centro el contorno en el origen
    wound_countour_mm = wound_countour_mm * pix_to_mm #Convierto de pixeles a milimetros
    wound_countour_mm = wound_countour_mm.reshape(wound_countour_mm.shape[0], 2)
    distance_matrix = squareform(pdist(wound_countour_mm))
    i, j = np.unravel_index(np.argmax(distance_matrix), distance_matrix.shape)

    point1 = np.array((wound_countour_mm[i][0], wound_countour_mm[i][1]))
    point2 = np.array((wound_countour_mm[j][0], wound_countour_mm[j][1]))
    length_axis = point2 - point1
    wound_length_axis_distance = np.linalg.norm(length_axis)
    print(wound_length_axis_distance)
    num_stitches_pairs = int(wound_length_axis_distance // 15)
    stitches_center = np.zeros((num_stitches_pairs - 1, 2))

    point_1_pix = point1 / pix_to_mm
    point_1_pix += wound_center_pixel

    point_2_pix = point2 / pix_to_mm
    point_2_pix += wound_center_pixel



    for k in range(1, num_stitches_pairs):
        stitches_center[k - 1] = point1 + (k/num_stitches_pairs) * length_axis
    
    stitches = np.zeros((num_stitches_pairs - 1,2,2))
    for m in range(len(stitches_center)):
        
        perp_vec = np.array([-length_axis[1], length_axis[0]])
        perp_vec = perp_vec/np.linalg.norm(perp_vec) #Vector perpendicular al eje normalizado
        actual_point = stitches_center[m]

        mean_sup_width, mean_inf_width = find_perpendicular_edge({"actual_point": actual_point,
         "wound_countour_mm": wound_countour_mm, "tolerance": 0.7, "length_axis": length_axis / wound_length_axis_distance})
        
        sup_point = actual_point + perp_vec * (mean_sup_width + 5)
        inf_point = actual_point + perp_vec * (mean_inf_width - 5)
        stitches[m] = np.array([sup_point, inf_point])

    sup_stitches = stitches[:,0]
    stitching_order, _ = order_stitches({"stitches": sup_stitches})
    
    ordered_stitches = stitches[stitching_order]
    
    stitches_pix = ordered_stitches / pix_to_mm
    stitches_pix += wound_center_pixel
    stitches_pix = stitches_pix.reshape(stitches_pix.shape[0]*2,1, 2)
    stitches_pix = stitches_pix.squeeze()

    point_1_pix = point1 / pix_to_mm
    point_1_pix += wound_center_pixel

    point_2_pix = point2 / pix_to_mm
    point_2_pix += wound_center_pixel

    print(f"wound length pix {np.linalg.norm(point_2_pix - point_1_pix)}")


    if SHOW:
        p1 = [int(point_1_pix[0]),int(point_1_pix[1])]
        p2 = [int(point_2_pix[0]),int(point_2_pix[1])]
        image = cv2.circle(image, p1, radius=3, color=(0, 255, 255), thickness=-1)
        image = cv2.circle(image, p2, radius=3, color=(0, 255, 255), thickness=-1)
        
        for stitch in stitches_pix:
            x = int(stitch[0])
            y = int(stitch[1])
            image = cv2.circle(image, [x,y], radius=3, color=(0, 0, 255), thickness=-1)
        cv2.imshow("stitch", image)
        cv2.waitKey(SHOW_TIME)
        if SAVE:
            save_image(image, "stitches")
    
    return stitches_pix

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

def save_image(image, img_name):


    cv2.imwrite(img_name, image)
    print(f"Saved image as: {img_name}")
