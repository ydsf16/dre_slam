// This file is part of dre_slam - Dynamic RGB-D Encoder SLAM for Differential-Drive Robot.
//
// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)
//
// dre_slam is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// dre_slam is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <object_detector.h>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>

using namespace cv;
using namespace dnn;
using namespace std;

Net net; // The yolov3 net.
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 320;  // Width of network's input image
int inpHeight = 320; // Height of network's input image
vector<string> classes;


// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess ( Mat& frame, const vector<Mat>& outs, std::vector< Object >& objects);

// Draw the predicted bounding box
void drawPred ( int classId, float conf, int left, int top, int right, int bottom, Mat& frame );

// Get the names of the output layers
vector<String> getOutputsNames ( const Net& net );


ObjectDetector::ObjectDetector ( const std::string& classes_file, const std::string& model_config, const std::string& model_weights )
{
    // Load names of classes.
    std::ifstream ifs ( classes_file.c_str() );
    string line;
    while ( getline ( ifs, line ) ) {
        classes_.push_back ( line );
    }

    // Assign to the global value.
    classes = classes_;

    // Load the network
    net = readNetFromDarknet ( model_config, model_weights );
    net.setPreferableBackend ( DNN_BACKEND_OPENCV );
	net.setPreferableTarget ( DNN_TARGET_CPU );
}


int ObjectDetector::detect ( const cv::Mat& imag_in, cv::Mat& image_out, std::vector< Object >& objects )
{
	image_out = imag_in.clone();
    cv::Mat blob;

	blobFromImage ( image_out, blob, 1/255.0, cv::Size ( inpWidth, inpHeight ), Scalar ( 0,0,0 ), true, false );

    //Sets the input to the network
    net.setInput ( blob );

    // Runs the forward pass to get output of the output layers
    vector<Mat> outs;
    net.forward ( outs, getOutputsNames ( net ) );

    // Remove the bounding boxes with low confidence
	postprocess ( image_out, outs, objects);
	
	return objects.size();
}


// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess ( Mat& frame, const vector<Mat>& outs, std::vector< Object >& objects)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for ( size_t i = 0; i < outs.size(); ++i ) {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = ( float* ) outs[i].data;
        for ( int j = 0; j < outs[i].rows; ++j, data += outs[i].cols ) {
            Mat scores = outs[i].row ( j ).colRange ( 5, outs[i].cols );
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc ( scores, 0, &confidence, 0, &classIdPoint );
            if ( confidence > confThreshold ) {
                int centerX = ( int ) ( data[0] * frame.cols );
                int centerY = ( int ) ( data[1] * frame.rows );
                int width = ( int ) ( data[2] * frame.cols );
                int height = ( int ) ( data[3] * frame.rows );
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back ( classIdPoint.x );
                confidences.push_back ( ( float ) confidence );
                boxes.push_back ( Rect ( left, top, width, height ) );
				
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes ( boxes, confidences, confThreshold, nmsThreshold, indices );
    for ( size_t i = 0; i < indices.size(); ++i ) {
        int idx = indices[i];
        Rect box = boxes[idx];
        drawPred ( classIds[idx], confidences[idx], box.x, box.y,
                   box.x + box.width, box.y + box.height, frame );
		
		// TODO changed by YDS.
		Object obj;
		obj.id_ = classIds[idx];
		obj.name_ = classes[obj.id_];
		obj.score_ = confidences[idx];
		obj.rect_box_ = box;
		objects.push_back(obj);
    }
}

// Draw the predicted bounding box
void drawPred ( int classId, float conf, int left, int top, int right, int bottom, Mat& frame )
{
    //Draw a rectangle displaying the bounding box
    rectangle ( frame, Point ( left, top ), Point ( right, bottom ), Scalar ( 255, 178, 50 ), 3 );

    //Get the label for the class name and its confidence
    string label = format ( "%.2f", conf );
    if ( !classes.empty() ) {
        CV_Assert ( classId < ( int ) classes.size() );
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize ( label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine );
    top = max ( top, labelSize.height );
    rectangle ( frame, Point ( left, top - round ( 1.5*labelSize.height ) ), Point ( left + round ( 1.5*labelSize.width ), top + baseLine ), Scalar ( 255, 255, 255 ), FILLED );
    putText ( frame, label, Point ( left, top ), FONT_HERSHEY_SIMPLEX, 0.75, Scalar ( 0,0,0 ),1 );
}

// Get the names of the output layers
vector<String> getOutputsNames ( const Net& net )
{
    static vector<String> names;
    if ( names.empty() ) {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize ( outLayers.size() );
        for ( size_t i = 0; i < outLayers.size(); ++i ) {
            names[i] = layersNames[outLayers[i] - 1];
        }
    }
    return names;
}
