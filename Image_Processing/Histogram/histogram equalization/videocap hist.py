import cv2
import numpy as np
from matplotlib import pyplot as plt

cap=cv2.VideoCapture("videoplayback.mp4")

while(True):

    ret_,img=cap.read()
    b,g,r=cv2.split(img)
    
    histb,binsb = np.histogram(b.flatten(),256,[0,256])
    cdfb = histb.cumsum()
    cdf_normalizedb = cdfb * histb.max()/ cdfb.max()
    
    histg,binsg = np.histogram(g.flatten(),256,[0,256])    
    cdfg = histg.cumsum()
    cdf_normalizedg = cdfg * histg.max()/ cdfg.max()
    
    histr,binsr = np.histogram(r.flatten(),256,[0,256])    
    cdfr = histr.cumsum()
    cdf_normalizedr = cdfr * histr.max()/ cdfr.max()
 
    cdf_mb = np.ma.masked_equal(cdfb,0)
    cdf_mb = (cdf_mb - cdf_mb.min())*255/(cdf_mb.max()-cdf_mb.min())
    cdfb = np.ma.filled(cdf_mb,0).astype('uint8')

    cdf_mg = np.ma.masked_equal(cdfg,0)
    cdf_mg = (cdf_mg - cdf_mg.min())*255/(cdf_mg.max()-cdf_mg.min())
    cdfg = np.ma.filled(cdf_mg,0).astype('uint8')

    cdf_mr = np.ma.masked_equal(cdfr,0)
    cdf_mr = (cdf_mr - cdf_mr.min())*255/(cdf_mr.max()-cdf_mr.min())
    cdfr = np.ma.filled(cdf_mr,0).astype('uint8')

    imgb = cdfb[b]
    imgg = cdfg[g]
    imgr = cdfr[r]


    equ=cv2.merge((imgb,imgg,imgr))


    cv2.imshow('img',img)
    cv2.imshow('equ',equ)

    cv2.waitKey(1)

cv2.destroyAllWindows()
