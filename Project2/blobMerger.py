#!/usr/bin/env python

import roslib
import rospy
import cv2
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
from std_msgs.msg import Int32

colorImage = Image()
isColorImageReady = False
blobsInfo = Blobs()
isBlobsInfoReady = False
bigPinkBlob = Blob()
bigRedBlob = Blob()

i = 0

pub = rospy.Publisher('blob_merger', Blobs, queue_size=10)

def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

def updateBlobsInfo(data):
    global blobsInfo, isBlobsInfoReady
    blobsInfo = data
    isBlobsInfoReady = True

def isWithin(blob1, blob2):
    if (blob1.right >= blob2.left and blob1.left <= blob2.right) and topBottom(blob1, blob2):
        return True
    elif (blob1.left <= blob2.right and blob1.right >= blob2.left) and topBottom(blob1, blob2):
        return True
    else:
        return False
    
def topBottom(blob1, blob2):
    return (blob2.bottom >= blob1.top and blob1.top >= blob2.top) or (blob1.top <= blob2.top and blob1.top >= blob2.bottom)

def filterBlob(blob):
    global bigPinkBlob,i,bigRedBlob
    if i == 0:
        i+=1
        return isWithin(blob, bigPinkBlob)
    elif i == 1:
        return isWithin(blob, bigRedBlob)

def mergeBlobs(blobs):
    global bigRedBlob,bigPinkBlob, pub
    x = 0
    y = 0
    left = 0
    right = 0
    top = 0
    bottom = 0
    area = 0
    
    pinkBlobs = []
    redBlobs = []
    
    for blob in blobs:
        if blob.name == "BallRed":
            redBlobs.append(blob)
        elif blob.name == "GoalPink":
            pinkBlobs.append(blob)

    # Sort the blobs and find the big blob
    bBA = 0
    for blob in redBlobs:
        if blob.area > bBA:
            bBA = blob.area
            bigRedBlob = blob
    bBA = 0
    for blob in pinkBlobs:
        if blob.area > bBA:
            bBA = blob.area
            bigPinkBlob = blob

    # Filter blobs, remove all blobs not within area
    pBlobs = filter(filterBlob, pinkBlobs)
    rBlobs = filter(filterBlob, redBlobs)
    if len(pBlobs) == 0: 
        pBlobs.append(bigPinkBlob)
    if len(rBlobs) == 0:
        rBlobs.append(bigRedBlob)
    
    firstPinkBlob = pBlobs[0]
    maxLeft = firstPinkBlob.left
    maxRight = firstPinkBlob.right
    maxTop = firstPinkBlob.top
    maxBottom = firstPinkBlob.bottom

    for blob in pBlobs:
        maxLeft = blob.left if blob.left < maxLeft else maxLeft
        maxRight = blob.right if blob.right > maxRight else maxRight
        maxTop = blob.top if blob.top < maxTop else maxTop
        maxBottom = blob.bottom if blob.bottom > maxBottom else maxBottom

    # Create new blob based on prior blobs (i.e. merged)
    pResult = Blob()
    pResult.x = (maxLeft + maxRight) / 2
    pResult.y = (maxTop + maxBottom) / 2 
    pResult.left = maxLeft
    pResult.right = maxRight
    pResult.top = maxTop
    pResult.bottom = maxBottom
    pResult.area = x * y
    pResult.name = firstPinkBlob.name
    
    firstRedBlob = rBlobs[0]
    maxLeft = firstRedBlob.left
    maxRight = firstRedBlob.right
    maxTop = firstRedBlob.top
    maxBottom = firstRedBlob.bottom

    for blob in rBlobs:
        maxLeft = blob.left if blob.left < maxLeft else maxLeft
        maxRight = blob.right if blob.right > maxRight else maxRight
        maxTop = blob.top if blob.top < maxTop else maxTop
        maxBottom = blob.bottom if blob.bottom > maxBottom else maxBottom
    
    rResult = Blob()
    rResult.x = (maxLeft + maxRight) / 2
    rResult.y = (maxTop + maxBottom) / 2
    rResult.left = maxLeft 
    rResult.right = maxRight
    rResult.top = maxTop
    rResult.bottom = maxBottom
    rResult.area = x * y
    rResult.name = firstRedBlob.name
    
    blobArray = Blobs()
    blobArray.blobs = [pResult, rResult]
    pub.publish(blobArray)
    return blobArray

def main():
    global colorImage, isColorImageReady, blobsInfo, isBlobsInfoReady
    rospy.init_node('showBlobs', anonymous=True)
    rospy.Subscriber("/blobs", Blobs, updateBlobsInfo)
    rospy.Subscriber("/v4l/camera/image_raw", Image, updateColorImage)
    bridge = CvBridge()
    cv2.namedWindow("Blob Location")

    while not rospy.is_shutdown() and (not isBlobsInfoReady or not isColorImageReady):
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"

        blobsCopy = copy.deepcopy(blobsInfo)

        if len(blobsCopy.blobs) > 0:
            blobber = mergeBlobs(blobsCopy.blobs)
            arr = blobber.blobs
            pinkBlob = arr[0]
            redBlob = arr[1]
            cv2.rectangle(color_image, (pinkBlob.left, pinkBlob.top),(pinkBlob.right, pinkBlob.bottom), (0,255,0), 2)
            cv2.rectangle(color_image, (redBlob.left,redBlob.top),(redBlob.right, redBlob.bottom), (0,255,0),2)
        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()
