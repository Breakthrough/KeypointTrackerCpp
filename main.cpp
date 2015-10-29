//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//            KeypointTrackerCpp: Video Keypoint Tracking                   //
//  ------------------------------------------------------------------      //
//        [ https://github.com/Breakthrough/KeypointTrackerCpp ]            //
//                                                                          //
//  This program implements full-search motion estimation to allow          //
//  tracking the movement/position of user-selected keypoints in either     //
//  a video file, or a live camera feed (webcam).                           //
//                                                                          //
//  Keypoints are added by clicking the left mouse button, and can be       //
//  removed with the right mouse button.  The keypoint size and search      //
//  window can currently be adjusted by modifying the constants in this     //
//  file.  Specifying a video source (file or cap.dev) is done in main.     //
//                                                                          //
//  Copyright (C) 2013-2015 Brandon Castellano < www.bcastell.com >.        //
//                                                                          //
//  KeypointTrackerCpp is licensed under the BSD 2-Clause License; see      //
//  the included LICENSE file or visit the following page for details:      //
//  https://github.com/Breakthrough/KeypointTrackerCpp                      //
//                                                                          //
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,         //
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF      //
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  //
//  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR       //
//  OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   //
//  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR   //
//  OTHER DEALINGS IN THE SOFTWARE.                                         //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc_c.h"

#include <stdio.h>
#include <time.h>
#include <list>

static CvScalar colors[] = 
{
    {{0,0,255}},
    {{255,0,0}},
    {{0,255,255}}
};


struct trackingPoint
{
    int cx, cy;
    int x0, y0;
};

std::list<trackingPoint> tpList;


void addTrackingPoint(int x, int y)
{
    trackingPoint toAdd;
    toAdd.cx = toAdd.x0 = x;
    toAdd.cy = toAdd.y0 = y;
    tpList.push_back(toAdd);
}


void removeTrackingPoint(int x, int y)
{
    // remove any tracking points within += 40px
    for (std::list<trackingPoint>::iterator it = tpList.begin();
        it != tpList.end(); it++)
    {
        if (abs(x - (*it).x0) <= 40 && abs(y - (*it).y0) <= 40)
            it = tpList.erase(it);
    }
}


void mouseHandler(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            addTrackingPoint(x, y);
            break;
        case CV_EVENT_RBUTTONDOWN:
            removeTrackingPoint(x, y);
            break;
        default:
            break;
    }
}

// p0_o and p1_o make up the rect in the origin image
// p0 and p1 (not req'd) are the points in i1.
int get_sq_fitness(CvPoint &p0_o, CvPoint &p1_o, CvPoint &p0, CvPoint &p1, IplImage *i0, IplImage *i1)
{
    
    //n[j + (i*ar)] = get_sq_fitness(p0_orig, p1_orig, p0, p1, f0, f1);
    int sosd  = 0;
    schar delta = 0;
    uchar* img_pt0;
    uchar* img_pt1;
    int i = 0;
    int j = 0;
    
    while (p0_o.x + i < p1_o.x)
    {
        while (p0_o.y + j < p1_o.y)
        {
            //img_pt0 = &((uchar*)(i0->imageData + i0->widthStep*(p0_o.y + j)))[(p0_o.x + i)*3];
            //img_pt1 = &((uchar*)(i1->imageData + i1->widthStep*(p0.y + j)))[(p0.x + i)*3];    
            img_pt0 = &((uchar*)(i0->imageData + i0->widthStep*(p0_o.y + j)))[(p0_o.x + i)*3];
            img_pt1 = &((uchar*)(i1->imageData + i1->widthStep*(p0.y + j)))[(p0.x + i)*3];    
            delta = img_pt0[0] - img_pt1[0];
            sosd += abs(delta); //(delta*delta);
            delta = img_pt0[1] - img_pt1[1];
            sosd += abs(delta); //(delta*delta);
            delta = img_pt0[2] - img_pt1[2];
            sosd += abs(delta); //(delta*delta);
            j++;
        }
        i++;
        j = 0;
    }
    
    
    return sosd;
}


// p0_o and p1_o make up the rect in the origin image
// p0 and p1 (not req'd) are the points in i1.
int get_sq_fitness_hist(CvPoint &p0_o, CvPoint &p1_o, CvPoint &p0, IplImage *i0, IplImage *i1)
{
    int h0[3][256];
    int h1[3][256];
    int hist_delta  = 0;
    int delta = 0;
    uchar* img_pt0;
    uchar* img_pt1;
    int i = 0;
    int j = 0;
    
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 256;j++)
        {
            h0[i][j] = 0;
            h1[i][j] = 0;
        }
    }
    i = 0;
    j = 0;
    
    while (p0_o.x + i < p1_o.x)
    {
        while (p0_o.y + j < p1_o.y)
        {    
            img_pt0 = &((uchar*)(i0->imageData + i0->widthStep*(p0_o.y + j)))[(p0_o.x + i)*3];
            img_pt1 = &((uchar*)(i1->imageData + i1->widthStep*(p0.y + j)))[(p0.x + i)*3];    
            h0[0][img_pt0[0]]++;
            h0[1][img_pt0[1]]++;
            h0[2][img_pt0[2]]++;
            h1[0][img_pt1[0]]++;
            h1[1][img_pt1[1]]++;
            h1[2][img_pt1[2]]++;
            j++;
        }
        i++;
        j = 0;
    }
    
    for (i = 0; i < 3; i++)
        for (j = 0; j < 256;j++)
            hist_delta += abs(h0[i][j] - h1[i][j]);
            
    return hist_delta;
}

int main( int argc, char** argv )
{
    cv::VideoCapture cap("sample.mkv");
    if (!cap.isOpened())
    {
        printf("Error - could not open capture file sample.mkv!");
        return 2;
    }
    
    bool pause = false;    
    
    
    IplImage* f0; // = cvLoadImage("lena.jpg",       CV_LOAD_IMAGE_COLOR);
    IplImage* f1; // = cvLoadImage("lena_moved.jpg", CV_LOAD_IMAGE_COLOR);
    
    
    cv::Mat effZero;
    cap.read(effZero);
    IplImage effIpl = effZero;
    f0 = cvCloneImage(&effIpl);
    
    /*int sx = 450;    //search_x  (i-(ss/2) to < (i+ss/2)
    int sy = 210;    //search_y
    
    int sx0 = sx;
    int sy0 = sy;*/
    cvNamedWindow( "original", 1 );
    int mouseParam = 5;
    cvSetMouseCallback("original",mouseHandler,&mouseParam);
    
    
    
    //addTrackingPoint(450, 210);
    //addTrackingPoint(750, 181);
    
    int fc = 1; // frame count
    double curr_fps = 0.0f;
    double frame_time = 0.0f;


    while (cap.isOpened())
    {
        time_t start, end;
        time(&start);

        if (!pause)
        {
            cap.read(effZero);
            effIpl = effZero;
            f1 = cvCloneImage(&effIpl);    
            fc++;
            
            int ss = 64;    //search_size (blue box)
            int ta = 20;    //(red box)
            int ar = ss - ta;
            
            
            ss /= 2;    
            /*
            if (fc % 30 == 0)
            {
                f0 = cvCloneImage(f1);
                for (std::list<trackingPoint>::iterator it = tpList.begin();
                    it != tpList.end(); it++)
                {
                    it->x0 = it->cx;
                    it->y0 = it->cy;
                }
                printf("KEYFRAME!\n");
            }*/
            
            for (std::list<trackingPoint>::iterator it = tpList.begin();
                it != tpList.end(); it++)
            {
                int min = -1;
                int min_x = it->cx;
                int min_y = it->cy;
                int *n = new int[ar*ar];
            
                CvPoint p0_orig = { it->x0 - (ta/2), it->y0 - (ta/2) };
                CvPoint p1_orig = { p0_orig.x + ta,     p0_orig.y + ta     };
                
                for (int i = 0; i < ar; i++)
                {
                    for (int j = 0; j < ar; j++)
                    {
                        CvPoint p0 = { (it->cx - ss) + i, (it->cy - ss) + j };
                        CvPoint p1 = { p0.x + ta,     p0.y + ta     };
                        //n[j + (i*ar)] = get_sq_fitness(p0_orig, p1_orig, p0, p1, f0, f1);
                        n[j + (i*ar)] = get_sq_fitness_hist(p0_orig, p1_orig, p0, f0, f1);

                        if (min < 0 || n[j + (i*ar)] < min)
                        {
                            min = n[j + (i*ar)];
                            min_x = i;
                            min_y = j;
                        }
                    }
                }
                
                
                // translate to pixel coords
                min_x = (it->cx - ss) + min_x + (ta/2);
                min_y = (it->cy - ss) + min_y + (ta/2);
                //printf("New coords & i = %d, j = %d\n", min_x, min_y);    
                
                CvPoint p0 = { it->cx, it->cy };
                CvPoint p1 = { it->cx, it->cy };
                p0.x += 7;    p0.y += 7;
                p1.x -= 8;    p1.y -= 8;
                //cvRectangle(outImg, p0, p1, colors[0], 2);
                cvRectangle(f1, p0, p1, colors[2], 2);
                p0.x += ar/2;    p0.y += ar/2;
                p1.x -= ar/2;    p1.y -= ar/2;
                //cvRectangle(outImg, p0, p1, colors[1], 2);
                cvRectangle(f1, p0, p1, colors[1], 2);
                
                p0.x = min_x - 7;    p0.y = min_y - 7;
                p1.x = min_x + 8;    p1.y = min_y + 8;
                cvRectangle(f1, p0, p1, colors[0], 2);
                        
                it->cx = min_x;
                it->cy = min_y;
                
                delete[] n;
                
                // bound check.
                if (it->cx < ss || it->cy < ss || it->cx > (f0->width - ss) || it->cy > (f0->height - ss))
                    it = tpList.erase(it);
            }
            
            // write frame.
            /*
            cv::Mat mater(f1);
            char fname[100];
            sprintf(fname, "./bcastell_ss/ant%04d.jpg", fc);
            cv::imwrite(fname, mater);*/
        }
        
        
        // Update coords.
        

        char fpstext[20];
        sprintf(fpstext, "FPS: %03d", curr_fps);
        CvFont font;
        double hScale=0.75;
        double vScale=0.75;
        int    lineWidth=1;
        cvInitFont(&font, CV_FONT_NORMAL, hScale,vScale,0,lineWidth);
        cvPutText(f1, fpstext, cvPoint(10,f0->height - 10), &font, cvScalar(0,0,255));

        cvShowImage( "original", f1 );
        




        int key = cvWaitKey(1);
        if (key > 0)
        {
            if (char(key) == 'q')
                break;
            if (char(key) == 'p')
                pause = !pause;
        }

        

        time(&end);
        frame_time += difftime(end, start);

        if (fc > 0 && !(fc % 5) && !pause) 
        {
            // compute curr_fps from frame_time
            curr_fps   = 5 / frame_time;
            frame_time = 0.0;
        }
        
        int number  = 5;
        char text[255]; 
        sprintf(text, "FPS: %03d", (int)number);
    }
        

    cvDestroyWindow( "original" );
    cvReleaseImage(&f0);
    cvReleaseImage(&f1);
    
}
