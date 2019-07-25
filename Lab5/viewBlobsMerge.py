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
bigBlob = Blob()

pub = rospy.Publisher('blob_move', Int32, queue_size=10)

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
    global bigBlob
    return isWithin(blob, bigBlob)

def mergeBlobs(blobs):
    global bigBlob, pub
    x = 0
    y = 0
    left = 0
    right = 0
    top = 0
    bottom = 0
    area = 0

    # Sort the blobs and find the big blob
    bBA = 0
    for blob in blobs:
        if blob.area > bBA:
            bBA = blob.area
            bigBlob = blob

    # Filter blobs, remove all blobs not within area
    blobs = filter(filterBlob, blobs)
    if len(blobs) == 0: 
        blobs.append(bigBlob)
    firstBlob = blobs[0]
    maxLeft = firstBlob.left
    maxRight = firstBlob.right
    maxTop = firstBlob.top
    maxBottom = firstBlob.bottom

    for blob in blobs:
        maxLeft = blob.left if blob.left < maxLeft else maxLeft
        maxRight = blob.right if blob.right > maxRight else maxRight
        maxTop = blob.top if blob.top < maxTop else maxTop
        maxBottom = blob.bottom if blob.bottom > maxBottom else maxBottom

    # Create new blob based on prior blobs (i.e. merged)
    result = Blob()
    result.x = (maxLeft + maxRight) / 2
    result.y = (maxTop + maxBottom) / 2 
    result.left = maxLeft
    result.right = maxRight
    result.top = maxTop
    result.bottom = maxBottom
    result.area = x * y

    pub.publish(result.x)
    print "Error: ", result.x-320
    return result

def main():
    global colorImage, isColorImageReady, blobsInfo, isBlobsInfoReady
    rospy.init_node('showBlobs', anonymous=True)
    rospy.Subscriber("/blobs", Blobs, updateBlobsInfo)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage)
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
            oneBlob = mergeBlobs(blobsCopy.blobs)
            cv2.rectangle(color_image, (oneBlob.left, oneBlob.top), (oneBlob.right, oneBlob.bottom), (0,255,0), 2)

        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()
