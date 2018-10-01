/**
 * @file test_detector.cpp
 * @author Jun Jin
 * @date 3 Mar 2017
 * @brief Implementation for the ArcDetector class
 *
 */
#include <au_vision/arc_detector.h>
extern "C"{
#include "au_vision/external/image.h"
#include "au_vision/external/pgm.h"
#include "au_vision/external/misc.h"
#include "au_vision/external/svg.h"
#include "au_vision/external/polygon.h"
#include "au_vision/external/ring.h"
#include "au_vision/external/elsdc.h"
}


namespace au_vision
{
	ArcDetector::ArcDetector(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
		: Detector( nh, private_nh )
	{
		detectorType_ = "arc_detector";
	}

ArcDetector::~ArcDetector()
{

}


std::vector<au_core::Roi> ArcDetector::detect(const cv::Mat &frame)
{
    std::vector<au_core::Roi> roiArray;
     bool isEffective=false;
     bool lockHLSS=false;

		 std::stringstream ss;
	   cv::Mat Src_L;
	   /// Convert image to gray and blur it
	   cv::Mat ViewControl=frame;
	   bool usingGray2View=false;//true: using gray image or HSL(L) image to view the result;  false: using RGB image to view the result
	   double BigCompareScale=0.35;//detection of the ellipse size, compared with the image's total width.
	   //decide to choose which color space.
	   //Src_L is used for final detection
	   if(isEffective||lockHLSS)
		 {
		   //convert image to Gray image. store in Src_L
			cv::cvtColor( frame, Src_L, CV_RGB2GRAY );
			if(usingGray2View)
				ViewControl=Src_L;
		 }
		else if(!lockHLSS)
		{
			//convert from RGB to HLS, store in src_HLS
			cv::cvtColor( frame, src_HLS, CV_RGB2HLS );
			std::vector <cv::Mat> channels(3);//store HLS channels
			//split the channels
			cv::split(src_HLS, channels);
			Src_L=channels[2];//Src_L = channel S in HLS
			if(usingGray2View)
			ViewControl=Src_L;
		}

		cv::blur(Src_L, Src_L, cv::Size(3,3));//blur the image firstly, can be removed for test purpose.

		PImageDouble in;     /* input image */
		  PImageInt    out;    /* output image having the same size as 'in'; the pixels
								  supporting a certain geometric primitive are marked
								  with the same label */

		  int ell_count = 0;   /* number of detected ellipses */
		  int *ell_labels=NULL;/* the pixels supporting a certain ellipse are marked
								  with the same unique label */
		  Ring *ell_out = NULL;/* array containing the parameters of the detected
								  ellipses; correlated with ell_labels, i.e. the i-th
								  element of ell_labels is the label of the pixels
								  supporting the ellipse defined by the parameters
								  ell[i] */

		  int poly_count = 0;  /* number of detected polygons */
		  int *poly_labels=NULL;/* the pixels supporting a certain polygon are marked
								  with the same unique label */
		  Polygon *poly_out=NULL;/* array containing the parameters of the detected
								  polygons; correlated with poly_labels, i.e. the i-th
								  element of ell_labels is the label of the pixels
								  supporting the polygon defined by the parameters
								  poly[i] */

		  FILE *ell_ascii;     /* output file with the parameters of the detected
								  ellipses -- ASCII format */
		  FILE *poly_ascii;    /* output file with the parameters of the detected
								  polygons -- ASCII format */
		  FILE *fsvg;          /* output file with the detected ellipses and polygons
								  in vectorial form */
		  int i,j;

		  /* read input image; must be PGM form */
		  in = read_pgm_image_double1(Src_L);
		  int xsize = in->xsize, ysize = in->ysize;

		  /* create and initialize with 0 output label image */
		  out = new_PImageInt_ini( in->xsize, in->ysize, 0 );

		  /* call detection procedure */
		  ELSDc( in, &ell_count, &ell_out, &ell_labels, &poly_count, &poly_out,
				 &poly_labels, out );

		 if( ell_out != NULL)
		{
			 std::vector<int> CandidatesIndexs;
			 double MaxABValue=0;
			 for( i=0; i<ell_count; i++ )//fill the candidates
			 {
				 double startAngle=ell_out[i].ang_start;
				 double endAngle=ell_out[i].ang_end;
				 double RadiusTempt=(ell_out[i].ax+ell_out[i].bx)/2;
				 if(endAngle>=(startAngle+1.8) ||((endAngle<startAngle)&& (endAngle+6.2832-startAngle)>1.8))
				 {
					double isValid=false;
					//Detect the reflection Cases.
					if(startAngle>=1.57&&startAngle<=4.712)
						isValid=true;
					else if(startAngle>=0&&startAngle<1.57)
					{
						if(endAngle<startAngle||endAngle>4.712)
							isValid=true;
					}
					else if(startAngle>4.712)
					{
						if(endAngle>startAngle||endAngle<0.78)
							isValid=true;
					}
					//End of detecting the reflection cases
					double frameWidth=frame.cols;
					if(isEffective&&(RadiusTempt>=0.25*frameWidth||RadiusTempt<=0.02*frameWidth)) //Radius should not be too large
						isValid=false;
					if(RadiusTempt>=0.04*frameWidth)
						lockHLSS=true;

					if(isValid)
					{
						 //capare the a, and b values
						if(RadiusTempt>MaxABValue)
							MaxABValue= RadiusTempt;
						CandidatesIndexs.push_back(i);
						int thisIndex=i;
					}
				 }//end of if (endAngle>=
			 }//end of for( i=0; i<ell_count; i++ )//fill the candidates

			isEffective=false;
			 for( i=0; i<CandidatesIndexs.size(); i++ )
			  {
				int thisIndex=CandidatesIndexs[i];
				 double thisABValue=(ell_out[thisIndex].ax+ell_out[thisIndex].bx)/2;
				 if(thisABValue>=BigCompareScale*MaxABValue)
				 {
					 //found the ROIs.
					 isEffective=true;

					 cv::Point2d startP, endP, centerP, centerStartP, centerEndP;
					 startP.x=ell_out[thisIndex].x1;
					 startP.y=ell_out[thisIndex].y1;
					endP.x=ell_out[thisIndex].x2;
					endP.y=ell_out[thisIndex].y2;
					centerP.x=ell_out[thisIndex].cx;
					centerP.y=ell_out[thisIndex].cy;

					centerStartP.x=centerP.x-ell_out[i].ax/2;
					centerStartP.y=centerP.y-ell_out[i].bx/2;
          double xoffset=centerStartP.x;
          double yoffset=centerStartP.y;
          double t_width=ell_out[i].ax;//std::abs(centerEndP.x-centerStartP.x);
          double t_height=ell_out[i].bx;//std::abs(centerEndP.y-centerStartP.y);
					ss << "buoy,";
					//decide the color, using HLS hue channel
					int cx=(int)(centerP.x);
					int cy=(int)(centerP.y);
					int B_value=0;//(frame.at<cv::Vec3b>(cv::Point(cx,cy)))[0];
					int G_value=0;//(frame.at<cv::Vec3b>(cv::Point(cx,cy)))[1];
					int R_value=0;//(frame.at<cv::Vec3b>(cv::Point(cx,cy)))[2];
					 B_value=(frame.at<cv::Vec3b>(cv::Point(cx,cy)))[0];
					 G_value=(frame.at<cv::Vec3b>(cv::Point(cx,cy)))[1];
					 R_value=(frame.at<cv::Vec3b>(cv::Point(cx,cy)))[2];
          std::string str_color="red";
					if(R_value>220&&G_value>220)
						str_color="yellow";
	         if(G_value>R_value&&G_value>B_value)
	              str_color="green";
	        ss<<str_color;
          au_core::Roi roi;
          roi.xOffset = xoffset;
          roi.yOffset = yoffset;
          roi.width = t_width;
          roi.height = t_height;
          roi.type = ss.str();
					roi.color=str_color;
          roiArray.push_back(roi);
				}// end of if
			  }// end of for
			  }//end of if( ell_out != NULL)
		 else
			 isEffective=false;
     return roiArray;
}

PImageDouble ArcDetector::read_pgm_image_double1(cv::Mat cv_image )
{
	PImageDouble image;
	/* get memory */
	int xsize=cv_image.size().width;
	int ysize=cv_image.size().height;
	  image = new_PImageDouble( xsize, ysize );

	  /* read data */

	  for(int y=0; y<ysize; y++)
	    for(int x=0; x<xsize; x++)
	    {
	    	cv::Scalar intensity=cv_image.at<uchar>(y, x);
	      image->data[ x + y * xsize ] = intensity.val[0];
	    }

	  return image;
}

} // end of namespace
