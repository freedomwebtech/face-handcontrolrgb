import cv2
from cvzone.HandTrackingModule import HandDetector
import board
import neopixel


np = neopixel.NeoPixel(board.D18, 60)
np.brightness = 0.5
    
detector=HandDetector(detectionCon=0.5,maxHands=1)
width=640
height=480
Known_distance = 60.0
Known_width = 9.0

cap=cv2.VideoCapture(0)
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

    focal_length = (width_in_rf_image * Known_distance ) / real_width 
    return focal_length
def obj_data(img):
    obj_width=0
    hands,frame=detector.findHands(img)
    if hands:
        hands1=hands[0]
        bbox=hands1["bbox"]
        x,y,w,h=bbox
        a.append([x,y])
        obj_width=w
    return obj_width
def obj_data1(img):
    hands=detector.findHands(img,draw=False)
    if hands:
        hands1=hands[0]
        print(hands1)
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
    frame=cv2.flip(frame,1)
    obj_width_in_frame=obj_data(frame)
    if not obj_width_in_frame:
        print("no hands")
        color6()
    else:    
        Distance = Distance_finder(Focal_length_found, Known_width, obj_width_in_frame)
        for i in a:
            x1=i[0]
            y1=i[1]
        cv2.putText(frame, f"Distance: {round(Distance,2)} CM", (x1,y1),cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2)
        if Distance < 60: 
            color1()
        elif 60 < Distance < 90:
             color2()
        elif 90 < Distance < 150:
            color3()
        elif 150 < Distance < 250:
            color5()
    
            
        
       
    
       
    cv2.imshow("FRAME",frame)
    
    if cv2.waitKey(1)&0xFF==27:
        break
cap.relase()
cv2.destroyAllWindows()
