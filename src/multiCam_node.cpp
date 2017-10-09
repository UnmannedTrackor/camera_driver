#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <pg_driver/Image.h>

using namespace std;

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap, unsigned int camNum)
{
  int result = 0;

  cout << "Printing device information for camera " << camNum << "..." << endl << endl;

  FeatureList_t features;
  CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
  if (IsAvailable(category) && IsReadable(category))
    {
      category->GetFeatures(features);

      FeatureList_t::const_iterator it;
      for (it = features.begin(); it != features.end(); ++it)
	{
	  CNodePtr pfeatureNode = *it;
	  cout << pfeatureNode->GetName() << " : ";
	  CValuePtr pValue = (CValuePtr)pfeatureNode;
	  cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
	  cout << endl;
	}
    }
  else
    {
      cout << "Device control information not available." << endl;
    }
  cout << endl;

  return result;
}


// Class for retrival PG driver's images
class ImagePub
{
public:
  ImagePub(ros::NodeHandle n)
    : n_(n), it_(n)
  {
    pub1_ = n_.advertise<pg_driver::Image>("image_pub1", 1000);
    pub2_ = n_.advertise<pg_driver::Image>("image_pub2", 1000);
  }

  virtual ~ImagePub() {}

  int AcquireImages(CameraList camList);
  int RunMultipleCameras(CameraList camList);


private:
  ros::NodeHandle n_;

  ros::Publisher pub1_;
  ros::Publisher pub2_;

};


// This function acquires and saves 10 images from each device.  
int ImagePub::AcquireImages(CameraList camList)
{
  int result = 0;
  CameraPtr pCam = NULL;

  cout << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

  try
    {
      //
      // Prepare each camera to acquire images
      // 
      // *** NOTES ***
      // For pseudo-simultaneous streaming, each camera is prepared as if it 
      // were just one, but in a loop. Notice that cameras are selected with 
      // an index. We demonstrate pseduo-simultaneous streaming because true 
      // simultaneous streaming would require multiple process or threads,
      // which is too complex for an example. 
      // 
      // Serial numbers are the only persistent objects we gather in this
      // example, which is why a vector is created.
      //
      vector<gcstring> strSerialNumbers(camList.GetSize());

      for (int i = 0; i < camList.GetSize(); i++)
	{
	  // Select camera
	  pCam = camList.GetByIndex(i);

	  // Set acquisition mode to continuous
	  CEnumerationPtr ptrAcquisitionMode = pCam->GetNodeMap().GetNode("AcquisitionMode");
	  if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
	    {
	      cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << i << "). Aborting..." << endl << endl;
	      return -1;
	    }

	  CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
	  if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
	    {
	      cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << i << "). Aborting..." << endl << endl;
	      return -1;
	    }

	  int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

	  ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

	  cout << "Camera " << i << " acquisition mode set to continuous..." << endl;

	  // Begin acquiring images
	  pCam->BeginAcquisition();

	  cout << "Camera " << i << " started acquiring images..." << endl;

	  // Retrieve device serial number for filename
	  strSerialNumbers[i] = "";

	  CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");

	  if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
	    {
	      strSerialNumbers[i] = ptrStringSerial->GetValue();
	      cout << "Camera " << i << " serial number set to " << strSerialNumbers[i] << "..." << endl;
	    }
	  cout << endl;
	}

      //
      // Retrieve, convert, and save images for each camera
      //
      // *** NOTES ***
      // In order to work with simultaneous camera streams, nested loops are
      // needed. It is important that the inner loop be the one iterating
      // through the cameras; otherwise, all images will be grabbed from a
      // single camera before grabbing any images from another.
      //

      // Retrieve, and publish images
      ros::Rate loop(1);
      std::cout << "Enter the loop" << std::endl;
      while(ros::ok()){

	for (int i = 0; i < camList.GetSize(); i++)
	  {
	    try
	      {
		// Select camera
		pCam = camList.GetByIndex(i);

		// Retrieve next received image and ensure image completion
		ImagePtr pResultImage = pCam->GetNextImage();

		if (pResultImage->IsIncomplete())
		  {
		    cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
		  }
		else
		  {
		    // Print image information
		    cout << "Camera " << i << " grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl;

		    // Convert image to mono 8
		    ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);

		    // Convert to CV mat
		    unsigned int XPadding = convertedImage->GetXPadding();
		    unsigned int YPadding = convertedImage->GetYPadding();
		    unsigned int rowsize = convertedImage->GetWidth();
		    unsigned int colsize = convertedImage->GetHeight();

		    cv::Mat mat= cv::Mat(colsize + YPadding, rowsize + XPadding,
					 CV_8UC1, convertedImage->GetData(), convertedImage->GetStride());
		    //msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
		    std::vector<uchar> array;
		    if (mat.isContinuous()) {
		      array.assign(mat.datastart, mat.dataend);
		    } else {
		      for (int i = 0; i < mat.rows; ++i) {
			array.insert(array.end(), mat.ptr<uchar>(i), mat.ptr<uchar>(i)+mat.cols);
		      }
		    }

		    pg_driver::ImagePtr img_out(new pg_driver::Image);
		    img_out.header.frame_id = "Image";
		    img_out.header.stamp = ros::Time::now();
		    img_out.data = array;
		    img_out.cols = mat.cols;
		    img_out.rows = mat.rows;
		    img_out.type = mat.type();
		    if (i==0)  pub1_.publish(img_out);
		    if (i==1)  pub2_.publish(img_out);
		  }

		// Release image
		pResultImage->Release();

		cout << endl;
	      }
	    catch (Spinnaker::Exception &e)
	      {
		cout << "Error: " << e.what() << endl;
		result = -1;
	      }
	  }

	ros::spinOnce();
        loop.sleep();
      }


      //
      // End acquisition for each camera
      //
      // *** NOTES ***
      // Notice that what is usually a one-step process is now two steps
      // because of the additional step of selecting the camera. It is worth
      // repeating that camera selection needs to be done once per loop.
      //
      // It is possible to interact with cameras through the camera list with
      // GetByIndex(); this is an alternative to retrieving cameras as 
      // CameraPtr objects that can be quick and easy for small tasks.
      //
      for (int i = 0; i < camList.GetSize(); i++)
	{
	  // End acquisition
	  camList.GetByIndex(i)->EndAcquisition();
	}
    }
  catch (Spinnaker::Exception &e)
    {
      cout << "Error: " << e.what() << endl;
      result = -1;
    }

  return result;
}

// This function acts as the body of the example; please see NodeMapInfo example 
// for more in-depth comments on setting up cameras.
int ImagePub::RunMultipleCameras(CameraList camList)
{
  int result = 0;
  CameraPtr pCam = NULL;

  try
    {
      //
      // Retrieve transport layer nodemaps and print device information for 
      // each camera
      //
      // *** NOTES ***
      // This example retrieves information from the transport layer nodemap 
      // twice: once to print device information and once to grab the device 
      // serial number. Rather than caching the nodemap, each nodemap is 
      // retrieved both times as needed.
      //
      cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

      for (int i = 0; i < camList.GetSize(); i++)
	{
	  // Select camera
	  pCam = camList.GetByIndex(i);

	  // Retrieve TL device nodemap
	  INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

	  // Print device information
	  result = PrintDeviceInfo(nodeMapTLDevice, i);
	}

      //
      // Initialize each camera
      // 
      // *** NOTES ***
      // You may notice that the steps in this function have more loops with
      // less steps per loop; this contrasts the AcquireImages() function 
      // which has less loops but more steps per loop. This is done for
      // demonstrative purposes as both work equally well.
      //
      // *** LATER ***
      // Each camera needs to be deinitialized once all images have been 
      // acquired.
      //
      for (int i = 0; i < camList.GetSize(); i++)
	{
	  // Select camera
	  pCam = camList.GetByIndex(i);

	  // Initialize camera
	  pCam->Init();
	}
		
      // Acquire images on all cameras
      result = result | AcquireImages(camList);

      // 
      // Deinitialize each camera
      //
      // *** NOTES ***
      // Again, each camera must be deinitialized separately by first
      // selecting the camera and then deinitializing it.
      //
      for (int i = 0; i < camList.GetSize(); i++)
	{
	  // Select camera
	  pCam = camList.GetByIndex(i);

	  // Deinitialize camera
	  pCam->DeInit();
	}
    }
  catch (Spinnaker::Exception &e)
    {
      cout << "Error: " << e.what() << endl;
      result = -1;
    }

  return result;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "driver");
  ros::NodeHandle nh;
  ros::Rate loop(1);    
 
  //======================================Camera Setup======================================//
  //======================================Camera Setup======================================//
  //======================================Camera Setup======================================//

  // Since this application saves images in the current folder
  // we must ensure that we have permission to write to this folder.
  // If we do not have permission, fail right away.
  FILE *tempFile = fopen("test.txt", "w+");
  if (tempFile == NULL)
    {
      cout << "Failed to create file in current folder.  Please check "
	"permissions."
	   << endl;
      cout << "Press Enter to exit..." << endl;
      getchar();
      return -1;
    }
  fclose(tempFile);
  remove("test.txt");

  // Print application build information
  cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

  // Retrieve singleton reference to system object
  SystemPtr system = System::GetInstance();

  // Retrieve list of cameras from the system
  CameraList camList = system->GetCameras();

  unsigned int numCameras = camList.GetSize();

  cout << "Number of cameras detected: " << numCameras << endl << endl;

  // Finish if there are no cameras
  if (numCameras == 0)
    {
      // Clear camera list before releasing system
      camList.Clear();

      // Release system
      system->ReleaseInstance();

      cout << "Not enough cameras!" << endl;
      cout << "Done! Press Enter to exit..." << endl;
      getchar();

      return -1;
    }

  ROS_INFO("Camera checked");

  // Run example on all cameras
  cout << endl << "Running example for all cameras..." << endl;
  ImageSub imageSub(nh);

  imageSub.RunMultipleCameras(camList);

  cout << "Example complete..." << endl << endl;

  // Clear camera list before releasing system
  camList.Clear();

  // Release system
  system->ReleaseInstance();
  
  cout << endl << "Done! Press Enter to exit..." << endl;
  getchar();

  return 0;
}

