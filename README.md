# ros-ariac


## Launch this project

Before launch this lab, launch the ariac_ws package first.

```
cd ~/ecse_373_ariac_ws/
source devel/setup.bash
roslaunch ecse_373_ariac ecse_373_ariac.launch
```

After ecse_373_ariac_ws has been successfully launched, use this command to launch cwru_ecse_373_submission:

```
cd ~/cwru_ecse_373_submission_ws 
source devel/setup.bash
roslaunch cwru_ecse_373_submission cwru_ecse_373_submission.launch
```

# Documention of how this lab works

## 1. Subscriber settings
Create 4 different types of subscriber. One for receiving orders information and the rest 3 for logical camera info. It is worth noting that, the logical camera subscriber callback function accept 2 parameters: msg and camera number. 

```
// Logical subscriber
for(int i=0; i < total_logical_camera_bin_num; i++)
{
    logical_camera_name = "/ariac/logical_camera_bin" + std::to_string(i);
    camera_sub[i] = n.subscribe<osrf_gear::LogicalCameraImage>(logical_camera_name, 10, boost::bind(binCameraCallback, _1, i));
}


// Callback function
void binCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int bin_camera_num)
{
    logic_camera_bin_vector[bin_camera_num] = *msg;
}
```

## 2. Service client settings
Create two different types of service client, one for detcting competition starts or not, the other for get product position infomation.

```
ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
ros::ServiceClient get_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
```

Use `call` interface to get the status result from service server. For the `start_competition` service, store the status result in variable `service_call_succeeded`, and if `service_call_succeeded==1` that means the competition started successfully, otherwise the competition failed to start.  Than use 'response' interface to get the content results from service server. The `/ariac/material_locations` are using a similar mechanism.

## 3. Finding the position of first order
From the order info which obtain from subscriber, we can get the first product info.
```
osrf_gear::Order first_order = order_vector[0];
osrf_gear::Shipment first_shipment = first_order.shipments[0];
osrf_gear::Product first_product = first_shipment.products[0];
```
Then use the product type of first order, and `/ariac/material_locations` service, we can figure out which bin the product store on. Then search through the all data obtained from logical bin camera subscriber to find the excatly position of first order.

## 4. Transform coordinate and fix the position
Once get the postion of first order, use `tf2::lookupTransform` and `tf2::doTransform` interface to transform the product position frame to arm frame. It is worth noting that we should find the frame of the product bin first. This will make the arm move to the part exactly and aviod collision, we should fix the product position a little bit.

