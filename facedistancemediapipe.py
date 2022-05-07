import cv2
import mediapipe as mp
from cvzone.HandTrackingModule import HandDetector
import board
import neopixel
from time import sleep

np = neopixel.NeoPixel(board.D18, 60)
np.brightness = 0.5
    
 
detector=HandDetector(detectionCon=0.5,maxHands=1)

mp_drawing = mp.solutions.drawing_utils
mp_face = mp.solutions.face_detection.FaceDetection(model_selection=1,min_detection_confidence=0.5)
cap=cv2.VideoCapture(0)
width=640
height=480
Known_distance = 66.0
Known_width = 14.0
a=[]
def color1():
     for i in range (60):
         color=[0,255,0]
         np[i]=(color)
         np.show()
def color2():
    for i in range(60):
        color=[255,0,0]
        np[i]=(color)
        np.show()

def color3():
    for i in range(60):
        color=[0,0,255]
        np[i]=(color)
        np.show()     
def color4():
     for i in range (60):  
         color=[0,255,255]
         np [i]=(color)
         np.show()
def color5():
    for i in range(60):
        color=[255,0,255]
        np[i]=(color)
        np.show()  
def color6():
    for i in range(60):
        color=[0,0,0]
        np[i]=(color)
        np.show()  
    
def Focal_Length_Finder(Known_distance, real_width, width_in_rf_image):

    focal_length = (width_in_rf_image * Known_distance) / real_width
    return focal_length

def obj_data(img):
    obj_width=0
    image_input = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = mp_face.process(image_input)
    if not results.detections:
       print("NO FACE")
    else: 
       for detection in results.detections:
           bbox = detection.location_data.relative_bounding_box
           x, y, w, h = int(bbox.xmin*width), int(bbox.ymin * height), int(bbox.width*width),int(bbox.height*height)
           cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
           a.append([x,y])
           obj_width=w
       return obj_width
def Distance_finder(Focal_Length, Known_width, obj_width_in_frame):
    distance = (Known_width * Focal_Length)/obj_width_in_frame
    return distance
ref_image = cv2.imread("rf.png")
ref_image_obj_width = obj_data(ref_image)
Focal_length_found = Focal_Length_Finder(Known_distance, Known_width, ref_image_obj_width)
cv2.imshow("ref_image", ref_image)

print(Focal_length_found)

while True:
    ret,frame=cap.read()
    frame=cv2.resize(frame,(640,480))
    frame=cv2.flip(frame,1)
    obj_width_in_frame=obj_data(frame)
    if not obj_width_in_frame:
        print("NO FACE")
    else:
        Distance = Distance_finder(Focal_length_found, Known_width, obj_width_in_frame)
        for i in a:
            x1=i[0]
            y1=i[1]
          
        cv2.putText(frame, f"Distance: {round(Distance,2)} CM", (x1, y1),cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2)
        if Distance < 60:
           hands,frame=detector.findHands(frame)
           if hands:
              hands1=hands[0]
              fingers1=detector.fingersUp(hands1)
              n=fingers1.count(1)
              print(n)
              if n ==1:
                 color2()
              elif n==2:
                 color1()
              elif n==3:
                 color3()
              elif n==4:
                 color4()
              elif n==5:
                  color5()
           if not hands:
              color6()      
        cv2.imshow("FRAME",frame)
    
    

    cv2.imshow("FRAME",frame)
    if cv2.waitKey(1)&0xFF==27:
        break
cap.release()
cv2.destroyAllWindows()

