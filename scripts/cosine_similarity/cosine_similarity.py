import numpy as np
import cv2 as cv


width = 800 
height = 600  
PUP1 = cv.imread("./pup1.png")
PUP1 = cv.resize(PUP1, (width, height))
PUP2 = cv.imread("./pup2.png")
PUP2 = cv.resize(PUP2, (width, height))
CAT1 = cv.imread("./cat.jpg")
CAT1 = cv.resize(CAT1, (width, height))




def calculate_cosine_similarity(A,B):
    gray_img1 = cv.cvtColor(A, cv.COLOR_BGR2GRAY)
    gray_img2 = cv.cvtColor(B, cv.COLOR_BGR2GRAY)
    A1 = gray_img1.flatten()
    B1 = gray_img2.flatten()
    dot_product = np.dot(A1, B1)

    norm_A = np.linalg.norm(A1)
    norm_B = np.linalg.norm(B1)

    return dot_product / (norm_A * norm_B)


print("Cosine Similarity between puppy 1 and puppy 2:", calculate_cosine_similarity(PUP1, PUP2))
print("Cosine Similarity between puppy 1 and cat:", calculate_cosine_similarity(PUP1, CAT1))
print("Cosine Similarity between puppy 2 and cat:", calculate_cosine_similarity(PUP2, CAT1))
print("Cosine Similarity between puppy 1 and puppy 1:", calculate_cosine_similarity(PUP1, PUP1))