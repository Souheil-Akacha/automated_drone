# -*- coding: utf-8 -*-
"""
Created on Mon Apr  8 19:10:49 2019

@author: Dr. Tarik Elamsy
"""
""" Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and downx.
"""
from djitellopy import Tello
import cv2
import pygame
from pygame.locals import *
import numpy as np
import time
import re
import string

# Speed of the drone
S = 60
# Frames per second of the pygame window display
FPS = 25


class IEEE_Drone(object):


    def __init__(self):
        # Init pygame
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10
        self.H=720
        self.W=960
        self.x0=480
        self.y0=360
        self.isDetectionOn=False
        self.isObjectDetected=False
        self.send_rc_control = False
        self.stopSearching=False
        self.isObjInCenter=False
        self.wasDetected=False
        self.objIsClose=False
        
        
        self.objID=None
        self.x=0
        self.y=0
        self.d=None
        self.Q=160
        self.Q2=90
        self.H_alignment=False
        self.V_alignment=False
 
        self.message="Starting"
        self.status="Detection mode:"+str(self.isDetectionOn)

        self.maxR=295
        self.minR=50
        self.lastmaxR=0
        
        self.frame=None
        self.screen1=None
        self.screen2=None

        self.Finalize=False
        self.total_rot=0
        self.rot_count=0
        self.showMask1=False
        self.showMask2=False
        self.searchCounter=0

        
        pygame.init()
        # Creat pygame window
        pygame.display.set_caption("RoboBug Team")
        
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        
        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(USEREVENT + 1, 50)
        pygame.time.set_timer(USEREVENT + 2, 15000)
        pygame.time.set_timer(USEREVENT + 3, 20000) #search again in different way
        


    def getNextFrame(self):
        frame_read = self.tello.get_frame_read()
        self.frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)


        
    def displayNextFrame(self):    

        cv2.putText(self.frame,"+",(self.x0,self.y0),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
        
        
        if self.isObjectDetected:
            cv2.putText(self.frame,"+",(self.x,self.y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
            cv2.putText(self.frame,"o",(self.x,self.y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
            
        if self.isDetectionOn:
            lineThickness = 2
            cv2.line(self.frame, (int(self.W/2),0), (int(self.W/2),self.H), (250,250,250), lineThickness)
            cv2.line(self.frame, (0,int(self.H/2)), (self.W,int(self.H/2)), (250,255,250), lineThickness)

        
        
        
        cv2.putText(self.frame,self.message,(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0))  
        cv2.putText(self.frame,self.status,(10,self.H-30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0)) 
        cv2.putText(self.frame,str("Battery:"+str(self.battery)),(self.W-130,self.H-30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0)) 
        self.frame = np.rot90(self.frame)
        self.frame = np.flipud(self.frame)
        frame = pygame.surfarray.make_surface(self.frame)              
        self.screen.fill([0, 0, 0])
        self.screen.blit(frame, (0, 0))
        pygame.display.update()
        
        
        
        

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            return

        if not self.tello.set_speed(self.speed):
            print("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            return
        
        self.getBattery()

        frame_read = self.tello.get_frame_read()

        self.should_stop = False
        while not self.should_stop:

            for event in pygame.event.get():
                if event.type == USEREVENT + 1:
                    self.update()
                elif event.type == USEREVENT + 2:
                    self.getBattery()
                    
                elif event.type == USEREVENT + 3 and not self.stopSearching:
                    self.search2()

                elif event.type == QUIT:
                    self.should_stop = True
                    
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                frame_read.stop()
                break
            
            self.getNextFrame()
            self.detection()
            
            if not self.Finalize:                
                if not self.stopSearching and self.send_rc_control and self.isDetectionOn:
                    self.search()
                elif self.objIsClose and self.stopSearching and self.isDetectionOn and self.isObjectDetected and  self.isHAligned() and  self.send_rc_control:
                    self.land()
                elif self.stopSearching and self.isDetectionOn and self.isObjectDetected and not self.isHAligned() and  self.send_rc_control:
                    self.HAlignTarget()
                elif self.stopSearching and self.isDetectionOn and self.isObjectDetected and not self.isVAligned() and  self.send_rc_control:
                    self.VAlignTarget()
                elif self.isHAligned() and self.isVAligned()  and self.send_rc_control:
                    self.goto()
            else: 
                self.should_stop=True
                break
      #          self.LandOnTarget()             
            self.displayNextFrame()
            time.sleep(1 / FPS)

        # Call it always before finishing. I deallocate resources.
        
        while True:
            if cv2.waitKey(1) ==ord('q'): #q to close the frame
                   break
            
        cv2.destroyAllWindows()
        self.tello.end()
        

        

    
  
    def filterRedObject(self):
    
           #             lowerBound=np.array([136,87,111],np.uint8) #red color
          #  upperBound=np.array([180,255,255],np.uint8) # red color upper bound
          #low HSV
           self.HSV= cv2.cvtColor(self.frame,cv2.COLOR_RGB2HSV)
           kernel=np.ones((5,5),np.uint8)
           Lower_red = np.array([0, 80, 80])
           Upper_red = np.array([10,250,250])
           Lower_red2 = np.array([160, 80, 80])
           Upper_red2 = np.array([180,250,250])
           Lower_red3 = np.array([173, 239,43])
           Upper_red3 = np.array([176,225,141])
           Lower_red4 = np.array([136, 87,111])
           Upper_red4 = np.array([180,250,250])
           
           mask1=cv2.inRange(self.HSV,Lower_red,Upper_red)
           mask2=cv2.inRange(self.HSV,Lower_red2,Upper_red2)
           mask3=cv2.inRange(self.HSV,Lower_red3,Upper_red3)
           mask4=cv2.inRange(self.HSV,Lower_red4,Upper_red4)
           mask=cv2.bitwise_or(mask1,mask2)
           #mask=cv2.bitwise_or(mask,mask3)
           #mask=cv2.bitwise_or(mask,mask4)
           
           #mask=cv2.bitwise_or(mask,mask4)
           #applying dilation and erode
           self.mask=mask
           
           self.maskE = cv2.erode(mask, kernel, iterations=2)
           self.maskM=cv2.morphologyEx(self.maskE,cv2.MORPH_OPEN,kernel)
           self.maskFinal = cv2.dilate(self.maskM, kernel, iterations=1)
           
           #cv2.imshow("mask", self.mask)
           #cv2.imshow("maskE",self.maskE)
           #cv2.imshow("mask M",self.maskM)

           #self.output=output_img[np.where(mask==0)] = 0
           

           cv2.imshow("mask Final",self.mask)
           # output=cv2.bitwise_and(self.frame.copy(),self.frame.copy(),mask=mask)
          # cv2.imshow("mask final",output)
           
           a=self.frame
           if self.showMask1:
               
               output1=cv2.bitwise_and(a.copy(),self.frame.copy(),mask=mask1)

              # cv2.imshow("mask 1",output1)
               output2=cv2.bitwise_and(a.copy(),self.frame.copy(),mask=mask2)

               #cv2.imshow("mask 2",output2)
               output3=cv2.bitwise_and(a.copy(),self.frame.copy(),mask=mask3)

               #cv2.imshow("mask 3",output3)
               output4=cv2.bitwise_and(a.copy(),self.frame.copy(),mask=mask4)

               #cv2.imshow("mask 4",output4)
    
               
               cv2.putText(output1,"mask 1",(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               cv2.putText(output2,"mask 2",(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               cv2.putText(output3,"mask Final",(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               cv2.putText(output4,"mask 4",(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               cv2.namedWindow("All masks",cv2.WINDOW_NORMAL)
               cv2.resizeWindow("All masks", self.W*2,self.H*2)
               vis1 = np.hstack((output1, output2))
               vis2 = np.hstack((output3, output4))        #vis = np.concatenate((img, img2_3channels), axis=1)
               vis = np.concatenate((vis1, vis2), axis=0)
               cv2.imshow("All masks",vis)
           else:
               cv2.destroyWindow("All masks")
               
           
           if self.showMask2:
               output5=cv2.bitwise_and(a.copy(),self.frame.copy(),mask=self.mask)

              # cv2.imshow("mask 1",output1)
               output6=cv2.bitwise_and(a.copy(),self.frame.copy(),mask=self.maskE)

               #cv2.imshow("mask 2",output2)
               output7=cv2.bitwise_and(a.copy(),self.frame.copy(),mask=self.maskM)

               #cv2.imshow("mask 3",output3)
               output8=cv2.bitwise_and(a.copy(),self.frame.copy(),mask=self.maskFinal)

               #cv2.imshow("mask 4",output4)
    
               
               cv2.putText(output5,"mask",(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               cv2.putText(output6,"mask E",(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               cv2.putText(output7,"mask Ml",(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               cv2.putText(output8,"mask Final",(30,30),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               cv2.namedWindow("image process",cv2.WINDOW_NORMAL)
               cv2.resizeWindow("image process", self.W*2,self.H*2)
               vis3 = np.hstack((output5, output6))
               vis4 = np.hstack((output7, output8))        #vis = np.concatenate((img, img2_3channels), axis=1)
               vis5 = np.concatenate((vis3, vis4), axis=0)
               cv2.imshow("image process",vis5)
               
           else:
               cv2.destroyWindow("image process")
               
               
           #maskClose=maskFinal=maskOpen


    def displayDetectedObject(self):
   
        if self.isObjectDetected and self.objID>-1:

            length=self.Q
            cv2.rectangle(self.frame,(self.x0-length,self.y0-length),(self.x0+length,self.y0+length),(255,255,255), 2)
            ((x, y), radius) = cv2.minEnclosingCircle(self.conts[self.objID])
            cv2.drawContours(self.frame,self.conts[self.objID],-1,(255,0,0),3)
            x=int(x)
            y=int(y)
            radius=int(radius)
            self.x=x
            self.y=y
            self.radius=radius
          
            cv2.rectangle(self.frame,(x-radius,y-radius),(x+radius,y+radius),(0,0,255), 2)
            cv2.circle(self.frame, (x,y), radius, (0,255,255), 2)
            position=("["+str(x)+","+str(y)+"]"+"Radius="+str(self.radius) )
            cv2.putText(self.frame,position,(x-30,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
            
 

                
    def detection(self):
        if not self.isDetectionOn:
            return
        
        self.filterRedObject()

        if self.isDetectionOn:
            conts,h=cv2.findContours(self.mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            self.conts=conts

            if not(len(conts)>0):
                if self.wasDetected:
                    print("Target lost... Reversing to locate it again.")
                    self.b(30)  #going back 30 cm
                    self.wasDetected=False
                    self.message="Target Lost..."
                    return
                else:
                    self.isObjectDetected=False
                    self.stopSearching=False
                    self.isObjInCenter=False
                    self.objID=-1
                    self.x=0
                    self.y=0
                    self.message="Target not determined"
                    pass
                
            else:

                minObjectRadiusSize=2
                maxRadius=0
                self.objID=-1
                for i in range(len(conts)):
                        x,y,w,h=cv2.boundingRect(conts[i])
                        ((x2, y2), radius) = cv2.minEnclosingCircle(conts[i])
                        
                        if radius>minObjectRadiusSize and radius>maxRadius :
                            self.objID=i
                            self.isObjectDetected=True
                            self.stopSearching=True
                            self.wasDetected=True
                            maxRadius=radius
                            if self.lastmaxR<radius:
                                self.lastmaxR=radius

                            

                            

        #            else:
     #                   self.objectDetected=False            
        self.displayDetectedObject()
                            

        
    def f(self,S):
        #self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,self.yaw_velocity
        self.tello.send_rc_control(0, int(S),0,0)
        self.message="Moving forward "+str(S)
    def b(self,S):
        #self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,self.yaw_velocity
        self.tello.send_rc_control(0,int(-S),0,0)
        self.message="Moving backward "+str(S)
    def l(self,S):
        #self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,self.yaw_velocity
        self.tello.send_rc_control(int(-S),0,0,0)
        self.message="Going left "+str(S)
    def r(self,S):
        #self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,self.yaw_velocity
        self.tello.send_rc_control(int(S),0,0,0)
        self.message="Going right "+str(S)
    def u(self,S):
        #self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,self.yaw_velocity
        self.tello.send_rc_control(0, 0,int(S),0)
        self.message="Ascending"+str(S)
    
    def down(self,S):     
        self.tello.send_rc_control(0, 0,int(-S),0)     
        self.message="Descending"+str(S)
        
    def rcw(self,S):
        #self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,self.yaw_velocity
        self.tello.send_rc_control(0,0,0,int(S))
        self.message="Rotating clockwise "+str(S)+" degrees"
        
    def rcc(self,S):
        #self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,self.yaw_velocity
        self.tello.send_rc_control(0,0,0,int(-S))  
        self.message="Rotating counter clockwise "+str(S)+" degrees"
        
    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw counter clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_k:  # set zero yaw velocity
            self.showMask1=not self.showMask1
            print("show image masks",self.showMask1)
                
        elif key == pygame.K_i:  # set zero yaw velocity
            self.showMask2=not self.showMask2
            print("show image processing",self.showMask2)
            
            
        elif key == pygame.K_a or key == pygame.K_o:  # set zero yaw velocity
            
            self.isDetectionOn=not  self.isDetectionOn
            self.isObjectDetected=False
            if self.isDetectionOn:
                self.message="Detection turned on"
            if not self.isDetectionOn:
                self.message="Detection turned off"
            print("Detection on",self.isDetectionOn)
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()

            self.send_rc_control = True
            self.message="Remote Control turned on"
            time.sleep(1)
            self.isDetectionOn=True

           #self.down(int(20))
            
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False
            self.message="Remote Control turned off"
            
    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)

   
        
    def search(self):
  #      if self.total_rot>720:
   #         self.tello.land()
    #        self.send_rc_control = False
     #       self.message="Remote Control turned off"
       
        print("Locating Object..")
        self.status="Searching Mode"
        self.message="Rotating counter clockwise 20 degrees"
        self.rcc(90)
        self.total_rot+=30
        self.rot_count+=1;
        
        
        
    def search2(self):
  #      if self.total_rot>720:
   #         self.tello.land()
    #        self.send_rc_control = False
     #       self.message="Remote Control turned off"
        self.searchCounter+=1
        
        if self.searchCounter%2==1:
           self.r(100)
           time.sleep(1)
            
        elif self.searchCounter%2==0:
            self.l(100)
            time.sleep(1)
            
            print("Locating Object..")
            self.status="Searching Mode"
            self.message="Rotating counter clockwise 20 degrees"
            self.rcc(30)
            self.total_rot+=30
            self.rot_count+=1;
        
 #   def AlignTarget(self):
    
    def isHAligned(self):
        if self.isObjectDetected and self.objID>-1:
            return  self.x in range(self.x0-self.Q-10,self.x0+self.Q+10)
        else:
            return False
   
    def isVAligned(self):
        if self.isObjectDetected and self.objID>-1:
            return  self.y in range(self.y0-self.Q,self.y0+self.Q)
        else:
            return False     
            
    def HAlignTarget(self):
        
         print("Horizontal Alignment Mode")
         self.status="H alignment Mode"
                  
         if self.x in range (0,self.Q):
             self.rcc(30)
             
         elif self.x in range (self.Q+1,self.Q*2):
             self.rcc(15)
             
         elif self.x in range (self.Q*2,int(self.Q*2+self.Q/3)):
             self.rcc(8)
        
         elif self.x in range(self.W-self.Q,self.W):
             self.rcw(30)

         elif self.x in range(self.W-self.Q*2,self.W-self.Q):
              self.rcw(15)
        
              
         elif self.x in range(self.W-int(self.Q*2+self.Q/3),self.W-self.Q*2):
             self.rcw(8)
   
             
         
         
    def VAlignTarget(self):
            
             print("Vertical Alignment Mode")
             print("(",self.x,",",self.y,") radius is ",self.radius)
             self.status="V Alignment Mode"
             if self.radius<self.maxR:
                 self.down(20)
                 time.sleep(1/10)
                 self.b(10)
                 self.lastmaxR-=5
                 
             if self.radius>self.maxR:
                     self.message="Target is in range... Commence landing procedure.."
                     self.objIsClose=True
                      
             elif self.y in range (self.y0+self.Q+1,self.H):
                 self.down(60)

             elif self.y in range (0,self.x0-self.Q-1):
                 self.u(30)

                 
    
             
    def goto(self):

        self.status="Navigating to target"
        
        if  self.radius in range (0,int(self.maxR/2)):
            self.f(65)
            time.sleep(1/11)
            self.message="Dashing forward 25"
            print(self.message)
       
        elif  self.radius in range (int(self.maxR/2)+1,self.maxR):
            self.f(45)
            time.sleep(1/11)
            self.message="Inching forward 14"
            print(self.message)
            
        elif self.radius>self.maxR :

            print("Target is in range... Commence landing procedure..")
            self.status="Landing on Target"  
            self.objIsClose=True

              
    def land(self):
            self.displayNextFrame()
            self.Finalize=True
            #move forward to center of target
            time.sleep(0.5)
            self.f(65)
            time.sleep(1)
            self.f(65)
            time.sleep(1)   
            self.tello.land()
            self.should_stop=True
            self.isDetectionOn=False
            self.isObjectDetected=False
            self.send_rc_control=False
            self.message="Mission Accomplished."
            
            self.displayNextFrame()
            
            
            
    def getBattery(self):
        response=str(self.tello.get_battery())
       # battery=re.search(r'\d+', string1).group()
        self.battery=str(response[:3])+str("%")
        print(self.battery)
        

                        
               
            
        
def main():
    myDrone = IEEE_Drone()

    # run frontend
    myDrone.run()


if __name__ == '__main__':
    main()
