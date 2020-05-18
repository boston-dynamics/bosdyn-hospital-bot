#include "spinnaker_sdk_camera_driver/capture.h"

acquisition::Capture::~Capture(){

    // destructor

    ifstream file(dump_img_.c_str());
    if (file)
        if (remove(dump_img_.c_str()) != 0)
            ROS_WARN_STREAM("Unable to remove dump image!");

    end_acquisition();
    deinit_cameras();

    // pCam = nullptr;
    
    ROS_INFO_STREAM("Clearing camList...");
    camList_.Clear();

    ROS_INFO_STREAM("Releasing camera pointers...");
    for (int i=0; i<cams.size(); i++)
        cams[i].~Camera();
    
    ROS_INFO_STREAM("Releasing system instance...");
    system_->ReleaseInstance();

    delete dynamicReCfgServer_;
    
    ros::shutdown();

}

void handler(int i) {

    // Capture* obj = reinterpret_cast<Capture*>(object);
    ROS_FATAL("HERE!!!");
    
}

acquisition::Capture::Capture(): it_(nh_), nh_pvt_ ("~") {

    // struct sigaction sigIntHandler;

    // sigIntHandler.sa_handler = handler;
    // sigemptyset(&sigIntHandler.sa_mask);
    // sigIntHandler.sa_flags = 0;

    // sigaction(SIGINT, &sigIntHandler, NULL);

    int mem;
    ifstream usb_mem("/sys/module/usbcore/parameters/usbfs_memory_mb");
    if (usb_mem) {
        usb_mem >> mem;
        if (mem >= 1000)
            ROS_INFO_STREAM("[ OK ] USB memory: "<<mem<<" MB");
        else{
	  ROS_FATAL_STREAM("  USB memory on system too low ("<<mem<<" MB)! Must be at least 1000 MB. Run: \nsudo sh -c \"echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb\"\n Terminating...");
            ros::shutdown();
        }
    } else {
        ROS_FATAL_STREAM("Could not check USB memory on system! Terminating...");
        ros::shutdown();
    }

    // default values for the parameters are set here. Should be removed eventually!!
    trigger_slave_from_master_ = false;
    exposure_time_ = 1000;
    gain_ = 0.0; // default as 0 = no gain
    soft_framerate_ = 20; //default soft framrate
    ext_ = ".bmp";
    SOFT_FRAME_RATE_CTRL_ = false;
    LIVE_ = false;
    TIME_BENCHMARK_ = false;
    MASTER_TIMESTAMP_FOR_ALL_ = true;    
    EXPORT_TO_ROS_ = false;
    PUBLISH_CAM_INFO_ = false;
    SAVE_ = false;
    SAVE_BIN_ = false;
    nframes_ = -1;
    FIXED_NUM_FRAMES_ = false;
    MAX_RATE_SAVE_ = false;
    rate_div_ = 1;
    rate_cut_ = 0;
    init_delay_ = 1; 
    master_fps_ = 20.0;
    binning_ = 1;
    todays_date_ = todays_date();

    dump_img_ = "dump" + ext_;

    grab_time_ = 0;
    save_time_ = 0;
    toMat_time_ = 0;
    save_mat_time_ = 0;
    export_to_ROS_time_ = 0; 
    achieved_time_ = 0;
        
    // decimation_ = 1;
  
    CAM_ = 0;

    // default flag values

    MANUAL_TRIGGER_ = false;
    CAM_DIRS_CREATED_ = false;

    GRID_CREATED_ = false;


    //read_settings(config_file);
    read_parameters();
    
    // Retrieve singleton reference to system object
    ROS_INFO_STREAM("Creating system instance...");
    system_ = System::GetInstance();

    load_cameras();

    enabled_ = true;
    Enable_ = nh_.advertiseService("camera_array_stream_enable", &Capture::onEnable, this);

    //initializing the ros publisher
    acquisition_pub = nh_.advertise<spinnaker_sdk_camera_driver::SpinnakerImageNames>("camera", 1000);
    //dynamic reconfigure
    dynamicReCfgServer_ = new dynamic_reconfigure::Server<spinnaker_sdk_camera_driver::spinnaker_camConfig>(nh_pvt_);
    
    dynamic_reconfigure::Server<spinnaker_sdk_camera_driver::spinnaker_camConfig>::CallbackType dynamicReCfgServerCB_t;   

    dynamicReCfgServerCB_t = boost::bind(&acquisition::Capture::dynamicReconfigureCallback,this, _1, _2);
    dynamicReCfgServer_->setCallback(dynamicReCfgServerCB_t);
}

acquisition::Capture::Capture(ros::NodeHandle nodehandl, ros::NodeHandle private_nh) : nh_ (nodehandl) , it_(nh_), nh_pvt_ (private_nh) {

    int mem;
    ifstream usb_mem("/sys/module/usbcore/parameters/usbfs_memory_mb");
    if (usb_mem) {
        usb_mem >> mem;
        if (mem >= 1000)
            ROS_INFO_STREAM("[ OK ] USB memory: "<<mem<<" MB");
        else{
            ROS_FATAL_STREAM("  USB memory on system too low ("<<mem<<" MB)! Must be at least 1000 MB. Run: \nsudo sh -c \"echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb\"\n Terminating...");
            ros::shutdown();
        }
    } else {
        ROS_FATAL_STREAM("Could not check USB memory on system! Terminating...");
        ros::shutdown();
    }

    // default values for the parameters are set here. Should be removed eventually!!
    trigger_slave_from_master_ = false;
    exposure_time_ = 1000;
    gain_ = 0.0; // default as 0 = no gain
    soft_framerate_ = 20; //default soft framrate
    ext_ = ".bmp";
    SOFT_FRAME_RATE_CTRL_ = false;
    LIVE_ = false;
    TIME_BENCHMARK_ = false;
    MASTER_TIMESTAMP_FOR_ALL_ = true;
    EXPORT_TO_ROS_ = false;
    PUBLISH_CAM_INFO_ = false;
    SAVE_ = false;
    SAVE_BIN_ = false;
    nframes_ = -1;
    FIXED_NUM_FRAMES_ = false;
    MAX_RATE_SAVE_ = false;
    rate_div_ = 1;
    rate_cut_ = 0;
    init_delay_ = 1;
    master_fps_ = 20.0;
    binning_ = 1;
    todays_date_ = todays_date();

    dump_img_ = "dump" + ext_;

    grab_time_ = 0;
    save_time_ = 0;
    toMat_time_ = 0;
    save_mat_time_ = 0;
    export_to_ROS_time_ = 0;
    achieved_time_ = 0;

    // decimation_ = 1;

    CAM_ = 0;

    // default flag values

    MANUAL_TRIGGER_ = false;
    CAM_DIRS_CREATED_ = false;

    GRID_CREATED_ = false;


    //read_settings(config_file);
    read_parameters();

    // Retrieve singleton reference to system object
    ROS_INFO_STREAM("Creating system instance...");
    system_ = System::GetInstance();

    load_cameras();

    enabled_ = true;
    Enable_ = nh_.advertiseService("camera_array_stream_enable", &Capture::onEnable, this);

    //initializing the ros publisher
    acquisition_pub = nh_.advertise<spinnaker_sdk_camera_driver::SpinnakerImageNames>("camera", 1000);
    //dynamic reconfigure
    dynamicReCfgServer_ = new dynamic_reconfigure::Server<spinnaker_sdk_camera_driver::spinnaker_camConfig>(nh_pvt_);
    
    dynamic_reconfigure::Server<spinnaker_sdk_camera_driver::spinnaker_camConfig>::CallbackType dynamicReCfgServerCB_t;   

    dynamicReCfgServerCB_t = boost::bind(&acquisition::Capture::dynamicReconfigureCallback,this, _1, _2);
    dynamicReCfgServer_->setCallback(dynamicReCfgServerCB_t);
}


void acquisition::Capture::load_cameras() {

    // Retrieve list of cameras from the system
    ROS_INFO_STREAM("Retreiving list of cameras...");
    camList_ = system_->GetCameras();
    
    numCameras_ = camList_.GetSize();
    ROS_ASSERT_MSG(numCameras_,"No cameras found!");
    ROS_INFO_STREAM("Numer of cameras found: " << numCameras_);
    ROS_INFO_STREAM(" Cameras connected: " << numCameras_);

    for (int i=0; i<numCameras_; i++) {
        acquisition::Camera cam(camList_.GetByIndex(i));
        ROS_INFO_STREAM("  -"<<cam.get_id());
       }

    bool master_set = false;
    int cam_counter = 0;
    
    
    for (int j=0; j<cam_ids_.size(); j++) {
        bool current_cam_found=false;
        for (int i=0; i<numCameras_; i++) {
        
            acquisition::Camera cam(camList_.GetByIndex(i));
            
            if (cam.get_id().compare(cam_ids_[j]) == 0) {
                current_cam_found=true;
                if (cam.get_id().compare(master_cam_id_) == 0) {
                    cam.make_master();
                    master_set = true;
                    MASTER_CAM_ = cam_counter;
                }
                
                ImagePtr a_null;
                pResultImages_.push_back(a_null);

                Mat img;
                frames_.push_back(img);
                time_stamps_.push_back("");
        
                cams.push_back(cam);
                
                camera_image_pubs.push_back(it_.advertiseCamera("camera_array/"+cam_names_[j]+"/image_raw", 70));
                //camera_info_pubs.push_back(nh_.advertise<sensor_msgs::CameraInfo>("camera_array/"+cam_names_[j]+"/camera_info", 1));

                img_msgs.push_back(sensor_msgs::ImagePtr());

                sensor_msgs::CameraInfoPtr ci_msg(new sensor_msgs::CameraInfo());
                int image_width = 0;
                int image_height = 0;
                std::string distortion_model = ""; 
                nh_pvt_.getParam("image_height", image_height);
                nh_pvt_.getParam("image_width", image_width);
                nh_pvt_.getParam("distortion_model", distortion_model);
                ci_msg->header.frame_id = "cam_"+to_string(j)+"_optical_frame";
                // full resolution image_size
                ci_msg->height = image_height;
                ci_msg->width = image_width;
                // distortion
                ci_msg->distortion_model = distortion_model;
                // binning
                ci_msg->binning_x = binning_;
                ci_msg->binning_y = binning_;
            
                if (PUBLISH_CAM_INFO_){
                    ci_msg->D = distortion_coeff_vec_[j];
                    // intrinsic coefficients
                    for (int count = 0; count<intrinsic_coeff_vec_[j].size();count++){
                        ci_msg->K[count] = intrinsic_coeff_vec_[j][count];
                    }
                    // Rectification matrix
                    if (!rect_coeff_vec_.empty()) 
                        ci_msg->R = {
                            rect_coeff_vec_[j][0], rect_coeff_vec_[j][1], 
                            rect_coeff_vec_[j][2], rect_coeff_vec_[j][3], 
                            rect_coeff_vec_[j][4], rect_coeff_vec_[j][5], 
                            rect_coeff_vec_[j][6], rect_coeff_vec_[j][7], 
                            rect_coeff_vec_[j][8]};
                    // Projection/camera matrix
                    if (!proj_coeff_vec_.empty()){
                        ci_msg->P = {
                            proj_coeff_vec_[j][0], proj_coeff_vec_[j][1], 
                            proj_coeff_vec_[j][2], proj_coeff_vec_[j][3], 
                            proj_coeff_vec_[j][4], proj_coeff_vec_[j][5], 
                            proj_coeff_vec_[j][6], proj_coeff_vec_[j][7], 
                            proj_coeff_vec_[j][8], proj_coeff_vec_[j][9], 
                            proj_coeff_vec_[j][10], proj_coeff_vec_[j][11]};
                    }
                    //else if(numCameras_ == 1){
                    else if(cam_ids_.size() == 1){  
                        // for case of monocular camera, P[1:3,1:3]=K
                        ci_msg->P = {
                        intrinsic_coeff_vec_[j][0], intrinsic_coeff_vec_[j][1],
                        intrinsic_coeff_vec_[j][2], 0, 
                        intrinsic_coeff_vec_[j][3], intrinsic_coeff_vec_[j][4],
                        intrinsic_coeff_vec_[j][5], 0, 
                        intrinsic_coeff_vec_[j][6], intrinsic_coeff_vec_[j][7],
                        intrinsic_coeff_vec_[j][8], 0};
                    }
                }

                cam_info_msgs.push_back(ci_msg);

                cam_counter++;
            
            }
        }
        if (!current_cam_found) ROS_WARN_STREAM("   Camera "<<cam_ids_[j]<<" not detected!!!");
    }
    ROS_ASSERT_MSG(cams.size(),"None of the connected cameras are in the config list!");
    ROS_ASSERT_MSG(master_set,"The camera supposed to be the master isn't connected!");
    // Setting numCameras_ variable to reflect number of camera objects used.
    // numCameras_ variable is used in other methods where it means size of cams list.
    numCameras_ = cams.size();
    // setting PUBLISH_CAM_INFO_ to true so export to ros method can publish it_.advertiseCamera msg with zero intrisics and distortion coeffs.
    PUBLISH_CAM_INFO_ = true;
}


void acquisition::Capture::read_parameters() {

    ROS_INFO_STREAM("*** PARAMETER SETTINGS ***");
    ROS_INFO_STREAM("** Date = "<<todays_date_);
    
    if (nh_pvt_.getParam("save_path", path_)){
    if(path_.front() =='~'){
        const char *homedir;
        if ((homedir = getenv("HOME")) == NULL)
            homedir = getpwuid(getuid())->pw_dir;
        std::string hd(homedir);
        path_.replace(0,1,hd);
    }
    ROS_INFO_STREAM("  Save path set via parameter to: " << path_);
    }
    else {
    boost::filesystem::path canonicalPath = boost::filesystem::canonical(".", boost::filesystem::current_path());
    path_ = canonicalPath.string();
       
    ROS_WARN_STREAM("  Save path not provided, data will be saved to: " << path_);
    }

    if (path_.back() != '/')
        path_ = path_ + '/';
        
    struct stat sb;
    ROS_ASSERT_MSG(stat(path_.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode),"Specified Path Doesn't Exist!!!");

    ROS_INFO("  Camera IDs:");
    
    std::vector<int> cam_id_vec;
    ROS_ASSERT_MSG(nh_pvt_.getParam("cam_ids", cam_id_vec),"If cam_aliases are provided, they should be the same number as cam_ids and should correspond in order!");
    int num_ids = cam_id_vec.size();
    for (int i=0; i < num_ids; i++){
        cam_ids_.push_back(to_string(cam_id_vec[i]));
        ROS_INFO_STREAM("    " << to_string(cam_id_vec[i]));
    }

    std::vector<string> cam_alias_vec;
    if (nh_pvt_.getParam("cam_aliases", cam_names_)){
        ROS_INFO_STREAM("  Camera Aliases:");
        ROS_ASSERT_MSG(num_ids == cam_names_.size(),"If cam_aliases are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<cam_names_.size(); i++) {
            ROS_INFO_STREAM("    " << cam_ids_[i] << " >> " << cam_names_[i]);
        }
    } else {
        ROS_INFO_STREAM("  No camera aliases provided. Camera IDs will be used as names.");
        for (int i=0; i<cam_ids_.size(); i++)
            cam_names_.push_back(cam_ids_[i]);
    }

    int mcam_int;
    ROS_ASSERT_MSG(nh_pvt_.getParam("master_cam", mcam_int),"master_cam is required!");
    master_cam_id_=to_string(mcam_int);
    bool found = false;
    for (int i=0; i<cam_ids_.size(); i++) {
        if (master_cam_id_.compare(cam_ids_[i]) == 0)
            found = true;
    }
    ROS_ASSERT_MSG(found,"Specified master cam is not in the cam_ids list!");
    
    if (nh_pvt_.getParam("utstamps", MASTER_TIMESTAMP_FOR_ALL_)){
        MASTER_TIMESTAMP_FOR_ALL_ = !MASTER_TIMESTAMP_FOR_ALL_;
        ROS_INFO("  Unique time stamps for each camera: %s",!MASTER_TIMESTAMP_FOR_ALL_?"true":"false");
    } 
        else ROS_WARN("  'utstamps' Parameter not set, using default behavior utstamps=%s",!MASTER_TIMESTAMP_FOR_ALL_?"true":"false");
    
    if (nh_pvt_.getParam("color", color_)) 
        ROS_INFO("  color set to: %s",color_?"true":"false");
        else ROS_WARN("  'color' Parameter not set, using default behavior color=%s",color_?"true":"false");

    if (nh_pvt_.getParam("to_ros", EXPORT_TO_ROS_)) 
        ROS_INFO("  Exporting images to ROS: %s",EXPORT_TO_ROS_?"true":"false");
        else ROS_WARN("  'to_ros' Parameter not set, using default behavior to_ros=%s",EXPORT_TO_ROS_?"true":"false");

    if (nh_pvt_.getParam("live", LIVE_)) 
        ROS_INFO("  Showing live images setting: %s",LIVE_?"true":"false");
        else ROS_WARN("  'live' Parameter not set, using default behavior live=%s",LIVE_?"true":"false");

    if (nh_pvt_.getParam("live_grid", GRID_VIEW_)){
        LIVE_=LIVE_|| GRID_VIEW_;
        ROS_INFO("  Showing grid-style live images setting: %s",GRID_VIEW_?"true":"false");
    } else ROS_WARN("  'live_grid' Parameter not set, using default behavior live_grid=%s",GRID_VIEW_?"true":"false");

    if (nh_pvt_.getParam("max_rate_save", MAX_RATE_SAVE_)) 
        ROS_INFO("  Max Rate Save Mode: %s",MAX_RATE_SAVE_?"true":"false");
        else ROS_WARN("  'max_rate_save' Parameter not set, using default behavior max_rate_save=%s",MAX_RATE_SAVE_?"true":"false");

    if (nh_pvt_.getParam("time", TIME_BENCHMARK_)) 
        ROS_INFO("  Displaying timing details: %s",TIME_BENCHMARK_?"true":"false");
        else ROS_WARN("  'time' Parameter not set, using default behavior time=%s",TIME_BENCHMARK_?"true":"false");

    if (nh_pvt_.getParam("rate_div", rate_div_)){
        if (rate_div_ > 0) ROS_INFO("  No. of images to rate_div set to: %d",rate_div_);
        else {
            rate_div_=1;
            ROS_WARN("  Provided 'rate_div' is not valid, using default behavior, rate_div=%d",rate_div_);
        }
    } else ROS_WARN("  'rate_div' Parameter not set, using default behavior: rate_div=%d",rate_div_);

    if (nh_pvt_.getParam("delay", init_delay_)){
        if (init_delay_>=0) ROS_INFO("  Init sleep delays set to : %0.2f sec",init_delay_);
        else {
            init_delay_=1;
            ROS_WARN("  Provided 'delay' is not valid, using default behavior, delay=%f",init_delay_);
        }
    } else ROS_WARN("  'delay' Parameter not set, using default behavior: delay=%f",init_delay_);

    if (nh_pvt_.getParam("fps", master_fps_)){
        if (master_fps_>=0) ROS_INFO("  Master cam fps set to : %0.2f",master_fps_);
        else {
            master_fps_=20;
            ROS_WARN("  Provided 'fps' is not valid, using default behavior, fps=%0.2f",master_fps_);
        }
    }
        else ROS_WARN("  'fps' Parameter not set, using default behavior: fps=%0.2f",master_fps_);

    if (nh_pvt_.getParam("exposure_time", exposure_time_)){
        if (exposure_time_ >0) ROS_INFO("  Exposure set to: %.1f",exposure_time_);
        else ROS_INFO("  'exposure_time'=%0.f, Setting autoexposure",exposure_time_);
    } else ROS_WARN("  'exposure_time' Parameter not set, using default behavior: Automatic Exposure ");

    if (nh_pvt_.getParam("trigger_slave_from_master", trigger_slave_from_master_)){
        if (trigger_slave_from_master_) {
            ROS_INFO("Trigger slave from master");
        } else {
            ROS_INFO("Free-run the slave cameras");
        }
    } else ROS_WARN("  'trigger_slave_from_master' Parameter not set, using default behavior: free-running slave cameras");

    if (nh_pvt_.getParam("gain", gain_)){
        if (gain_ >=0.0) ROS_INFO("  Gain set to: %.1f",gain_);
        else ROS_INFO("  'gain'=%0.f, Setting autogain",gain_);
    } else ROS_WARN("  'gain' Parameter not set, using default behavior: Automatic Gain ");

    if (nh_pvt_.getParam("target_grey_value", target_grey_value_)){
        if (target_grey_value_ >0) ROS_INFO("  target_grey_value set to: %.1f",target_grey_value_);
        else ROS_INFO("  'target_grey_value'=%0.f, Setting AutoExposureTargetGreyValueAuto to Continuous/ auto",target_grey_value_);} 
    else ROS_WARN("  'target_grey_value' Parameter not set, using default behavior: AutoExposureTargetGreyValueAuto to auto");


    if (nh_pvt_.getParam("binning", binning_)){
        if (binning_ >0) ROS_INFO("  Binning set to: %d",binning_);
        else {
            binning_=1;
            ROS_INFO("  'binning'=%d invalid, Using defauly binning=",binning_);
        }
    } else ROS_WARN("  'binning' Parameter not set, using default behavior: Binning = %d",binning_);

    if (nh_pvt_.getParam("soft_framerate", soft_framerate_)){
        if (soft_framerate_ >0) {
            SOFT_FRAME_RATE_CTRL_=true;
            ROS_INFO("  Using Software rate control, rate set to: %d",soft_framerate_);
        }
        else ROS_INFO("  'soft_framerate'=%d, software rate control set to off",soft_framerate_);
    }
    else ROS_WARN("  'soft_framerate' Parameter not set, using default behavior: No Software Rate Control ");

    if (nh_pvt_.getParam("save", SAVE_)) 
        ROS_INFO("  Saving images set to: %d",SAVE_);
        else ROS_WARN("  'save' Parameter not set, using default behavior save=%d",SAVE_);

    if (SAVE_||LIVE_){
        if (nh_pvt_.getParam("save_type", ext_)){
            if (ext_.compare("bin") == 0) SAVE_BIN_ = true;
            ROS_INFO_STREAM("    save_type set as: "<<ext_);
            ext_="."+ext_;
        }else ROS_WARN("    'save_type' Parameter not set, using default behavior save=%d",SAVE_);
    }

    if (SAVE_||MAX_RATE_SAVE_){
        if (nh_pvt_.getParam("frames", nframes_)) {
            if (nframes_>0){
                FIXED_NUM_FRAMES_ = true;
                ROS_INFO("    Number of frames to be recorded: %d", nframes_ );
            }else ROS_INFO("    Frames will be recorded until user termination");
        }else ROS_WARN("    'frames' Parameter not set, using defult behavior, frames will be recorded until user termination");
    }

    bool intrinsics_list_provided = false;
    XmlRpc::XmlRpcValue intrinsics_list;
    if (nh_pvt_.getParam("intrinsic_coeffs", intrinsics_list)) {
        ROS_INFO("  Camera Intrinsic Paramters:");
        ROS_ASSERT_MSG(intrinsics_list.size() == num_ids,"If intrinsic_coeffs are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<intrinsics_list.size(); i++){
            std::vector<double> intrinsics;
            String intrinsics_str="";
            for (int j=0; j<intrinsics_list[i].size(); j++){
                ROS_ASSERT_MSG(intrinsics_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                intrinsics.push_back(static_cast<double>(intrinsics_list[i][j]));
                intrinsics_str = intrinsics_str +to_string(intrinsics[j])+" ";
            }

            intrinsic_coeff_vec_.push_back(intrinsics);
            ROS_INFO_STREAM("   "<< intrinsics_str );
            intrinsics_list_provided=true;
        }
    }
    bool distort_list_provided = false;
    XmlRpc::XmlRpcValue distort_list;

    if (nh_pvt_.getParam("distortion_coeffs", distort_list)) {
        ROS_INFO("  Camera Distortion Paramters:");
        ROS_ASSERT_MSG(distort_list.size() == num_ids,"If intrinsic_coeffs are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<distort_list.size(); i++){
            std::vector<double> distort;
            String distort_str="";
            for (int j=0; j<distort_list[i].size(); j++){
                ROS_ASSERT_MSG(distort_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                distort.push_back(static_cast<double>(distort_list[i][j]));
                distort_str = distort_str +to_string(distort[j])+" ";
            }
            distortion_coeff_vec_.push_back(distort);
            ROS_INFO_STREAM("   "<< distort_str );
            distort_list_provided = true;
        }
    }
    
    XmlRpc::XmlRpcValue rect_list;

    if (nh_pvt_.getParam("rectification_coeffs", rect_list)) {
        ROS_INFO("  Camera Rectification Paramters:");
        ROS_ASSERT_MSG(rect_list.size() == num_ids,"If rectification_coeffs are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<rect_list.size(); i++){
            std::vector<double> rect;
            String rect_str="";
            for (int j=0; j<rect_list[i].size(); j++){
                ROS_ASSERT_MSG(rect_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                rect.push_back(static_cast<double>(rect_list[i][j]));
                rect_str = rect_str +to_string(rect[j])+" ";
            }
            rect_coeff_vec_.push_back(rect);
            ROS_INFO_STREAM("   "<< rect_str );
        }
    }
    
    XmlRpc::XmlRpcValue proj_list;

    if (nh_pvt_.getParam("projection_coeffs", proj_list)) {
        ROS_INFO("  Camera Projection Paramters:");
        ROS_ASSERT_MSG(proj_list.size() == num_ids,"If projection_coeffs are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<proj_list.size(); i++){
            std::vector<double> proj;
            String proj_str="";
            for (int j=0; j<proj_list[i].size(); j++){
                ROS_ASSERT_MSG(proj_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                proj.push_back(static_cast<double>(proj_list[i][j]));
                proj_str = proj_str +to_string(proj[j])+" ";
            }
            proj_coeff_vec_.push_back(proj);
            ROS_INFO_STREAM("   "<< proj_str );
        }
    }

    PUBLISH_CAM_INFO_ = intrinsics_list_provided && distort_list_provided;
    if (PUBLISH_CAM_INFO_)
        ROS_INFO("  Camera coeffs provided, camera info messges will be published.");
    else
        ROS_WARN("  Camera coeffs not provided correctly, camera info messges intrinsics and distortion coeffs will be published with zeros.");

//    ROS_ASSERT_MSG(my_list.getType()
//    int num_ids = cam_id_vec.size();
//    for (int i=0; i < num_ids; i++){
//        cam_ids_.push_back(to_string(cam_id_vec[i]));
//        ROS_INFO_STREAM("    " << to_string(cam_id_vec[i]));
//    }

}


void acquisition::Capture::init_array() {
    
    ROS_INFO_STREAM("*** FLUSH SEQUENCE ***");

    init_cameras(true);

    start_acquisition();
    sleep(init_delay_*0.5);

    end_acquisition();
    sleep(init_delay_*0.5);

    deinit_cameras();
    sleep(init_delay_*2.0);

    init_cameras(false);

    ROS_DEBUG_STREAM("Flush sequence done.");

}

void acquisition::Capture::init_cameras(bool soft = false) {

    ROS_INFO_STREAM("Initializing cameras...");
    
    // Set cameras 1 to 4 to continuous
    for (int i = numCameras_-1 ; i >=0 ; i--) {
                                
        ROS_DEBUG_STREAM("Initializing camera " << cam_ids_[i] << "...");

        try {
            
            cams[i].init();

            if (!soft) {

                cams[i].set_color(color_);
                /*cams[i].setIntValue("BinningHorizontal", binning_);
                  cams[i].setIntValue("BinningVertical", binning_);*/

                cams[i].setEnumValue("ExposureMode", "Timed");
                if (exposure_time_ > 0) { 
                    cams[i].setEnumValue("ExposureAuto", "Off");
                    cams[i].setFloatValue("ExposureTime", exposure_time_);
                } else {
                    cams[i].setEnumValue("ExposureAuto", "Continuous");
                }
                if (gain_ >= 0.0) { 
                    cams[i].setEnumValue("GainAuto", "Off");
                    cams[i].setFloatValue("Gain", gain_);
                } else {
                    cams[i].setEnumValue("GainAuto", "Continuous");
                }
                /*if (target_grey_value_ > 4.0) {
                    cams[i].setEnumValue("AutoExposureTargetGreyValueAuto", "Off");
                    cams[i].setFloatValue("AutoExposureTargetGreyValue", target_grey_value_);
                } else {
                    cams[i].setEnumValue("AutoExposureTargetGreyValueAuto", "Continuous");
                    }*/

                // cams[i].setIntValue("DecimationHorizontal", decimation_);
                // cams[i].setIntValue("DecimationVertical", decimation_);
                //cams[i].setBoolValue("AcquisitionFrameRateEnable", true);
                //cams[i].setFloatValue("AcquisitionFrameRate", 35.0);

                if (color_)
                    cams[i].setEnumValue("PixelFormat", "BGR8");
                else
                    cams[i].setEnumValue("PixelFormat", "Mono8");

                if (!trigger_slave_from_master_) {
                    cams[i].setEnumValue("AcquisitionMode", "Continuous");
                    cams[i].setEnumValue("TriggerMode", "Off");
                } else {
                    // set only master to be software triggered
                    if (cams[i].is_master()) {
                        cams[i].setEnumValue("AcquisitionMode", "Continuous");
                        cams[i].setEnumValue("TriggerMode", "Off");

                        cams[i].setEnumValue("LineSelector", "Line2");
                        cams[i].setEnumValue("LineMode", "Output");
                        cams[i].setEnumValue("LineSource", "ExposureActive");

                    } else {
                        cams[i].setEnumValue("TriggerMode", "On");
                        cams[i].setEnumValue("TriggerSource", "Line3");
                        cams[i].setEnumValue("TriggerSelector", "FrameStart");
                        //cams[i].setEnumValue("LineSelector", "Line3");
                        //cams[i].setEnumValue("LineMode", "Input");
                        cams[i].setEnumValue("TriggerOverlap", "ReadOut");

                        /*cams[i].setFloatValue("TriggerDelay", 40.0);
                        cams[i].setEnumValue("TriggerActivation", "RisingEdge");*/
                    }
                }
            }
        }

        catch (Spinnaker::Exception &e) {
            string error_msg = e.what();
            ROS_FATAL_STREAM("Error: " << error_msg);
            if (error_msg.find("Unable to set PixelFormat to BGR8") >= 0)
              ROS_WARN("Most likely cause for this error is if your camera can't support color and your are trying to set it to color mode");
            ros::shutdown();
        }

    }
    ROS_DEBUG_STREAM("All cameras initialized.");
}

bool acquisition::Capture::onEnable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  cout << "enable service callback before lock\n";
  const lock_guard<mutex> lock(enabled_mutex_);
  cout << "enable service callback " << (req.data ? 1 : 0)  << "\n";
  if (req.data) {
    start_acquisition();
    enabled_ = true;
  } else {
    enabled_ = false;
    end_acquisition();
  }
  res.success = true;
  return true;
}

void acquisition::Capture::start_acquisition() {

    for (int i = numCameras_-1; i>=0; i--)
        cams[i].begin_acquisition();

    // for (int i=0; i<numCameras_; i++)
    //     cams[i].begin_acquisition();
    
}

void acquisition::Capture::end_acquisition() {

    for (int i = 0; i < numCameras_; i++)
        cams[i].end_acquisition();
    
}

void acquisition::Capture::deinit_cameras() {

    ROS_INFO_STREAM("Deinitializing cameras...");

    // end_acquisition();
    
    for (int i = numCameras_-1 ; i >=0 ; i--) {

        ROS_DEBUG_STREAM("Camera "<<i<<": Deinit...");
        cams[i].deinit();
        // pCam = NULL;
    }
    ROS_INFO_STREAM("All cameras deinitialized."); 

}

void acquisition::Capture::create_cam_directories() {

    ROS_DEBUG_STREAM("Creating camera directories...");
    
    for (int i=0; i<numCameras_; i++) {
        ostringstream ss;
        ss<<path_<<cam_names_[i];
        if (mkdir(ss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0) {
            ROS_WARN_STREAM("Failed to create directory "<<ss.str()<<"! Data will be written into pre existing directory if it exists...");
        }
    }

    CAM_DIRS_CREATED_ = true;
    
}

void acquisition::Capture::save_mat_frames(int dump) {
    
    double t = ros::Time::now().toSec();

    if (!CAM_DIRS_CREATED_)
        create_cam_directories();
    
    string timestamp;
    for (unsigned int i = 0; i < numCameras_; i++) {

        if (dump) {
            
            imwrite(dump_img_.c_str(), frames_[i]);
            ROS_DEBUG_STREAM("Skipping frame...");
            
        } else {

            if (MASTER_TIMESTAMP_FOR_ALL_)
                timestamp = time_stamps_[MASTER_CAM_];
            else
                timestamp = time_stamps_[i];

            ostringstream filename;
            filename<< path_ << cam_names_[i] << "/" << timestamp << ext_;
            ROS_DEBUG_STREAM("Saving image at " << filename.str());
            //ros image names 
            mesg.name.push_back(filename.str());
            imwrite(filename.str(), frames_[i]);
            
        }

    }
    
    save_mat_time_ = ros::Time::now().toSec() - t;
    
}

void acquisition::Capture::export_to_ROS() {
    double t = ros::Time::now().toSec();
    std_msgs::Header img_msg_header;
    img_msg_header.stamp = mesg.header.stamp;

    for (unsigned int i = 0; i < numCameras_; i++) {
        img_msg_header.frame_id = "cam_"+to_string(i)+"_optical_frame";

        if(color_)
            img_msgs[i]=cv_bridge::CvImage(img_msg_header, "bgr8", frames_[i]).toImageMsg();
        else
            img_msgs[i]=cv_bridge::CvImage(img_msg_header, "mono8", frames_[i]).toImageMsg();

        if (PUBLISH_CAM_INFO_){
            cam_info_msgs[i]->header.stamp = mesg.header.stamp;
        }
        camera_image_pubs[i].publish(img_msgs[i],cam_info_msgs[i]);
/*
        if (PUBLISH_CAM_INFO_){
        cam_info_msgs[i].header.stamp = mesg.header.stamp;
        camera_info_pubs[i].publish(cam_info_msgs[i]);
        }
    */
    }
    export_to_ROS_time_ = ros::Time::now().toSec()-t;;
}

void acquisition::Capture::save_binary_frames(int dump) {
    
    double t = ros::Time::now().toSec();

    if (!CAM_DIRS_CREATED_)
        create_cam_directories();
    
    string timestamp;
    for (unsigned int i = 0; i < numCameras_; i++) {

        if (dump) {
            //imwrite(dump_img_.c_str(), frames_[i]);
            ROS_DEBUG_STREAM("Skipping frame...");
        } else {

            if (MASTER_TIMESTAMP_FOR_ALL_)
                timestamp = time_stamps_[MASTER_CAM_];
            else
                timestamp = time_stamps_[i];
                
            ostringstream filename;
            filename<< path_ << cam_names_[i] << "/" << timestamp << ".bin";
            ROS_DEBUG_STREAM("Saving image at " << filename.str());
            //ros image names
            mesg.name.push_back(filename.str());
            std::ofstream ofs(filename.str());
            boost::archive::binary_oarchive oa(ofs);
            oa << frames_[i];
            ofs.close();
            
        }

    }
    save_mat_time_ = ros::Time::now().toSec() - t;
    
}

void acquisition::Capture::get_mat_images() {
    //ros time stamp creation
    mesg.header.stamp = ros::Time::now();
    mesg.time = ros::Time::now();
    double t = ros::Time::now().toSec();
    
    ostringstream ss;
    ss<<"frameIDs: [";
    
    int frameID;
    int fid_mismatch = 0;

    const lock_guard<mutex> lock(enabled_mutex_);
    if (!enabled_) {
        return;
    }

    for (int i=0; i<numCameras_; i++) {
        //ROS_INFO_STREAM("CAM ID IS "<< i);
        frames_[i] = cams[i].grab_mat_frame();
        //ROS_INFO("sucess");
        time_stamps_[i] = cams[i].get_time_stamp();


        if (i==0)
            frameID = cams[i].get_frame_id();
        else
            if (cams[i].get_frame_id() != frameID)
                fid_mismatch = 1;
        
        if (i == numCameras_-1)
            ss << cams[i].get_frame_id() << "]";
        else
            ss << cams[i].get_frame_id() << ", ";
        
    }
    string message = ss.str();
    ROS_DEBUG_STREAM(message);

    if (fid_mismatch)
        ROS_WARN_STREAM("Frame IDs for grabbed set of images did not match!");
    
    toMat_time_ = ros::Time::now().toSec() - t;
    
}

void acquisition::Capture::run_soft_trig() {
    achieved_time_ = ros::Time::now().toSec();
    ROS_INFO("*** ACQUISITION ***");
    
    start_acquisition();

    // Camera directories created at first save
    
    if (LIVE_)namedWindow("Acquisition", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);

    int count = 0;
    
    //cams[MASTER_CAM_].trigger();
    get_mat_images();
    if (SAVE_) {
        count++;
        if (SAVE_BIN_)
            save_binary_frames(0);
        else
            save_mat_frames(0);
    }

    ros::Rate ros_rate(soft_framerate_);
    try{
        while( ros::ok() ) {

            bool enabled = true;
            {
                const lock_guard<mutex> lock(enabled_mutex_);
                enabled = enabled_;
            }
            if (!enabled) {
                ros_rate.sleep();
                continue;
            }

            double t = ros::Time::now().toSec();

            if (LIVE_) {
                if (GRID_VIEW_) {
                    update_grid();
                    imshow("Acquisition", grid_);
                } else {
                    imshow("Acquisition", frames_[CAM_]);
                    char title[50];
                    sprintf(title, "cam # = %d, cam ID = %s, cam name = %s", CAM_, cam_ids_[CAM_].c_str(), cam_names_[CAM_].c_str());
                    displayOverlay("Acquisition", title);
                }
            }

            int key = cvWaitKey(1);
            ROS_DEBUG_STREAM("Key press: "<<(key & 255)<<endl);
            
            if ( (key & 255)!=255 ) {

                if ( (key & 255)==83 ) {
                    if (CAM_<numCameras_-1) // RIGHT ARROW
                        CAM_++;
                } else if( (key & 255)==81 ) { // LEFT ARROW
                    if (CAM_>0)
                        CAM_--;
                } else if( (key & 255)==84 && MANUAL_TRIGGER_) { // t
		  //cams[MASTER_CAM_].trigger();
                    get_mat_images();
                } else if( (key & 255)==32 && !SAVE_) { // SPACE
                    ROS_INFO_STREAM("Saving frame...");
                    if (SAVE_BIN_)
                        save_binary_frames(0);
                        else{
                            save_mat_frames(0);
                            if (!EXPORT_TO_ROS_){
                                ROS_INFO_STREAM("Exporting frames to ROS...");
                                export_to_ROS();
                            }
                        }
                } else if( (key & 255)==27 ) {  // ESC
                    ROS_INFO_STREAM("Terminating...");
                    cvDestroyAllWindows();
                    ros::shutdown();
                    break;
                }
                ROS_DEBUG_STREAM("active cam switched to: "<<CAM_);
            }

            double disp_time_ = ros::Time::now().toSec() - t;

            // Call update functions
            if (!MANUAL_TRIGGER_) {
	      //cams[MASTER_CAM_].trigger();
                get_mat_images();
            }

            if (SAVE_) {
                count++;
                if (SAVE_BIN_)
                    save_binary_frames(0);
                else
                    save_mat_frames(0);
            }

            if (FIXED_NUM_FRAMES_) {
                cout<<"Nframes "<< nframes_<<endl;
                if (count > nframes_) {
                    ROS_INFO_STREAM(nframes_ << " frames recorded. Terminating...");
                    cvDestroyAllWindows();
                    break;
                }
            }
            
	    rate_cut_++;
	    if (rate_cut_ >= rate_div_) {
	      if (EXPORT_TO_ROS_) export_to_ROS();
	      rate_cut_ = 0;
	    }
            //cams[MASTER_CAM_].targetGreyValueTest();
            // ros publishing messages
            acquisition_pub.publish(mesg);

            // double total_time = grab_time_ + toMat_time_ + disp_time_ + save_mat_time_;
            double total_time = toMat_time_ + disp_time_ + save_mat_time_+export_to_ROS_time_;
            achieved_time_ = ros::Time::now().toSec() - achieved_time_;

            ROS_INFO_COND(TIME_BENCHMARK_,
                          "total time (ms): %.1f \tPossible FPS: %.1f\tActual FPS: %.1f",
                          total_time*1000,1/total_time,1/achieved_time_);
            
            ROS_INFO_COND(TIME_BENCHMARK_,"Times (ms):- grab: %.1f, disp: %.1f, save: %.1f, exp2ROS: %.1f",
                          toMat_time_*1000,disp_time_*1000,save_mat_time_*1000,export_to_ROS_time_*1000);
            
            achieved_time_=ros::Time::now().toSec();
            
            //if (SOFT_FRAME_RATE_CTRL_) {ros_rate.sleep();}

        }
    }
    catch(const std::exception &e){
        ROS_FATAL_STREAM("Excption: "<<e.what());
    }
    catch(...){
        ROS_FATAL_STREAM("Some unknown exception occured. \v Exiting gracefully, \n  possible reason could be Camera Disconnection...");
    }
}

float acquisition::Capture::mem_usage() {
    std::string token;
    std::ifstream file("/proc/meminfo");
    unsigned long int total, free;
    while (file >> token) {
        if (token == "MemTotal:")
            if (!(file >> total))
                ROS_FATAL_STREAM("Could not poll total memory!");
        if (token == "MemAvailable:")
            if (!(file >> free)) {
                ROS_FATAL_STREAM("Could not poll free memory!");
                break;
            }
        // ignore rest of the line
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return 1-float(free)/float(total);
}

void acquisition::Capture::update_grid() {

    if (!GRID_CREATED_) {
        int height = frames_[0].rows;
        int width = frames_[0].cols*cams.size();
        
        if (color_)
        grid_.create(height, width, CV_8UC3);
        else
        grid_.create(height, width, CV_8U);
        
        GRID_CREATED_ = true;
    }

    for (int i=0; i<cams.size(); i++)
        frames_[i].copyTo(grid_.colRange(i*frames_[i].cols,i*frames_[i].cols+frames_[i].cols).rowRange(0,grid_.rows));
    
}

//*** CODE FOR MULTITHREADED WRITING
void acquisition::Capture::write_queue_to_disk(queue<ImagePtr>* img_q, int cam_no) {
 
    ROS_DEBUG("  Write Queue to Disk Thread Initiated for cam: %d", cam_no);

    int imageCnt =0;
    string id = cam_ids_[cam_no];

    int k_numImages = nframes_;

    while (imageCnt < k_numImages){
//     ROS_DEBUG_STREAM("  Write Queue to Disk for cam: "<< cam_no <<" size = "<<img_q->size());

        if(img_q->empty()){
            boost::this_thread::sleep(boost::posix_time::milliseconds(5));
            continue;
        }

        ROS_DEBUG_STREAM("  Write Queue to Disk for cam: "<< cam_no <<" size = "<<img_q->size());

        if (img_q->size()>100)
            ROS_WARN_STREAM("  Queue "<<cam_no<<" size is :"<< img_q->size());

        ImagePtr convertedImage = img_q->front();
        uint64_t timeStamp =  convertedImage->GetTimeStamp() * 1000;

        // Create a unique filename
        ostringstream filename;
        filename<<path_<<cam_names_[cam_no]<<"/"<<cam_names_[cam_no]
                <<"_"<<id<<"_"<<todays_date_ << "_"<<std::setfill('0')
                << std::setw(6) << imageCnt<<"_"<<timeStamp << ext_; 
            
//     ROS_DEBUG_STREAM("Writing to "<<filename.str().c_str());

        convertedImage->Save(filename.str().c_str());
     
        queue_mutex_.lock();
        img_q->pop();
        queue_mutex_.unlock();

        ROS_DEBUG_STREAM("Image saved at " << filename.str());
        imageCnt++;
    }
}

void acquisition::Capture::acquire_images_to_queue(vector<queue<ImagePtr>>*  img_qs) {
    int result = 0;
    
    ROS_DEBUG("  Acquire Images to Queue Thread Initiated");
    start_acquisition();
    ROS_DEBUG("  Acquire Images to Queue Thread -> Acquisition Started");
    
    // Retrieve, convert, and save images for each camera
    
    int k_numImages = nframes_;
    auto start = ros::Time::now().toSec();
    auto elapsed = (ros::Time::now().toSec() - start)*1000;

    for (int imageCnt = 0; imageCnt < k_numImages; imageCnt++) {
        uint64_t timeStamp = 0;
        for (int i = 0; i < numCameras_; i++) {
            try {
                ImagePtr pResultImage = cams[i].grab_frame();

                // Convert image to mono 8
                ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);

                if(cams[i].is_master()) {
                    mesg.header.stamp = ros::Time::now();
                }
                timeStamp =  convertedImage->GetTimeStamp() * 1000;
                // Create a unique filename
                ostringstream filename;
                //filename << cam_ids_[i].c_str()<< "-" << imageCnt << ext_;
                filename << cam_names_[i]<<"_"<<cam_ids_[i].c_str()
                         << "_"<<todays_date_ << "_"<< std::setfill('0') 
                         << std::setw(6) << imageCnt<<"_"<<timeStamp << ext_;
                imageNames.push_back(filename.str());

                queue_mutex_.lock();
                img_qs->at(i).push(convertedImage);
                queue_mutex_.unlock();

                ROS_DEBUG_STREAM("Queue no. "<<i<<" size: "<<img_qs->at(i).size());

                // Release image
                pResultImage->Release();
            }
            catch (Spinnaker::Exception &e) {
                ROS_ERROR_STREAM("  Exception in Acquire to queue thread" << "\nError: " << e.what());
                result = -1;
            }
            if(i==0) {
                elapsed = (ros::Time::now().toSec() - start)*1000;
                start = ros::Time::now().toSec();
                //cout << "Microsecs passed: " << microseconds << endl;
                ROS_DEBUG_STREAM("Rate of cam 0 write to queue: " << 1e3/elapsed);
            }
        }
        mesg.name = imageNames;
        //make sure that the vector has no image file names
        imageNames.clear();

        // ros publishing messages
        acquisition_pub.publish(mesg);
    }
    return;
}

void acquisition::Capture::run_mt() {
    ROS_INFO("*** ACQUISITION MULTI-THREADED***");
    
    if (!CAM_DIRS_CREATED_)
        create_cam_directories();
    
    boost::thread_group threads;

    vector<std::queue<ImagePtr>> image_queue_vector;

    for (int i=0; i<numCameras_; i++) {
        std::queue<ImagePtr> img_ptr_queue;
        image_queue_vector.push_back(img_ptr_queue);
    }

    threads.create_thread(boost::bind(&Capture::acquire_images_to_queue, this, &image_queue_vector));

    for (int i=0; i<numCameras_; i++)
        threads.create_thread(boost::bind(&Capture::write_queue_to_disk, this, &image_queue_vector.at(i), i));

    ROS_DEBUG("Joining all threads");
    threads.join_all();
    ROS_DEBUG("All Threads Joined");
}

void acquisition::Capture::run() {
    if(!MAX_RATE_SAVE_)
        run_soft_trig();
    else
        run_mt();
}

std::string acquisition::Capture::todays_date()
{
    char out[9];
    std::time_t t=std::time(NULL);
    std::strftime(out, sizeof(out), "%Y%m%d", std::localtime(&t));
    std::string td(out);
    return td;
}

void acquisition::Capture::dynamicReconfigureCallback(spinnaker_sdk_camera_driver::spinnaker_camConfig &config, uint32_t level){
    
    ROS_INFO_STREAM("Dynamic Reconfigure: Level : " << level);
    if(level & 1){
        ROS_INFO_STREAM("Target grey value : " << config.target_grey_value);
        for (int i = numCameras_-1 ; i >=0 ; i--) {
            
            cams[i].setEnumValue("AutoExposureTargetGreyValueAuto", "Off");
            cams[i].setFloatValue("AutoExposureTargetGreyValue", config.target_grey_value);
        }
    }
    if (level & (1 << 1)){
        ROS_INFO_STREAM("Exposure "<<config.exposure_time);
        if(config.exposure_time > 0){
            for (int i = numCameras_-1 ; i >=0 ; i--) {

                cams[i].setEnumValue("ExposureAuto", "Off");
                cams[i].setEnumValue("ExposureMode", "Timed");
                cams[i].setFloatValue("ExposureTime", config.exposure_time);
            }
        }
        else if(config.exposure_time ==0){
            for (int i = numCameras_-1 ; i >=0 ; i--) {
                cams[i].setEnumValue("ExposureAuto", "Continuous");
                cams[i].setEnumValue("ExposureMode", "Timed");
            }
        }
    }
    for (int i = numCameras_-1 ; i >=0 ; i--) {
        if (level & (1 << (2+i))) {
  	    double gain = gain_;
	    switch (i) {
		case 0:
		    gain = config.gain_0;
		    break;
		case 1:
		    gain = config.gain_1;
		    break;
		case 2:
		    gain = config.gain_2;
		    break;
	    }

  	    ROS_INFO_STREAM("Level " << level << "; Gain " << i << " " << gain);
	    if(gain >= 0.0){
		cams[i].setEnumValue("GainAuto", "Off");
		cams[i].setFloatValue("Gain", gain);
	    }
	    else {
		cams[i].setEnumValue("GainAuto", "Continuous");
	    }
	}
    }
}
