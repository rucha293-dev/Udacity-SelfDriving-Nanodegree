
#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>
#include <carla/client/Vehicle.h>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono>
#include <ctime>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>

// helper.h confirms:
//   getPose(Eigen::Matrix4d)  -> Pose       (uses Matrix4d)
//   transform3D(...)          -> Matrix4d
// So ALL our matrices must be Matrix4d.
// PCL ICP/NDT internally use float, so we cast when calling them.

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;
int locMode = 2; // 0=ICP  1=NDT  2=NDT+ICP hybrid (default)

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{
    if (event.getKeySym() == "Right" && event.keyDown())
        cs.push_back(ControlState(0, -0.02, 0));
    else if (event.getKeySym() == "Left" && event.keyDown())
        cs.push_back(ControlState(0, 0.02, 0));
    if (event.getKeySym() == "Up" && event.keyDown())
        cs.push_back(ControlState(0.1, 0, 0));
    else if (event.getKeySym() == "Down" && event.keyDown())
        cs.push_back(ControlState(-0.1, 0, 0));
    if (event.getKeySym() == "a" && event.keyDown())
        refresh_view = true;
    if (event.getKeySym() == "o" && event.keyDown()){ locMode = 0; cout << "[Mode] ICP only" << endl; }
    if (event.getKeySym() == "n" && event.keyDown()){ locMode = 1; cout << "[Mode] NDT only" << endl; }
    if (event.getKeySym() == "h" && event.keyDown()){ locMode = 2; cout << "[Mode] NDT+ICP hybrid" << endl; }
}

void Accuate(ControlState response, cc::Vehicle::Control& state)
{
    if (response.t > 0){
        if (!state.reverse){ state.throttle = min(state.throttle+response.t, 1.0f); }
        else { state.reverse = false; state.throttle = min(response.t, 1.0f); }
    } else if (response.t < 0){
        response.t = -response.t;
        if (state.reverse){ state.throttle = min(state.throttle+response.t, 1.0f); }
        else { state.reverse = true; state.throttle = min(response.t, 1.0f); }
    }
    state.steer = min(max(state.steer+response.s, -1.0f), 1.0f);
    state.brake = response.b;
}

void drawCar(Pose pose, int num, Color color, double alpha,
             pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    BoxQ box;
    box.bboxTransform  = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4; box.cube_width = 2; box.cube_height = 2;
    renderBox(viewer, box, num, color, alpha);
}

// ─────────────────────────────────────────────────────────────────────────────
// Build 4x4 Matrix4d from a Pose
// Uses helper.h's transform3D which returns Matrix4d — we mirror that here
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix4d poseToMatrix(const Pose& p)
{
    // transform3D is defined in helper.cpp and returns Matrix4d
    return transform3D(p.rotation.yaw, p.rotation.pitch, p.rotation.roll,
                       p.position.x,   p.position.y,     p.position.z);
}

// ─────────────────────────────────────────────────────────────────────────────
// ICP alignment — returns Matrix4d (same type as getPose input)
// PCL icp.align() needs float internally, so we cast in/out
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix4d icpAlign(PointCloudT::Ptr source,
                          PointCloudT::Ptr target,
                          Eigen::Matrix4d initGuess,
                          int    maxIter = 60,
                          double maxDist = 4.0)
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(maxIter);
    icp.setMaxCorrespondenceDistance(maxDist);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    PointCloudT aligned;
    // PCL needs float — cast initGuess to float for the call
    icp.align(aligned, initGuess.cast<float>());

    if (icp.hasConverged()){
        cout << "[ICP] converged  fitness=" << icp.getFitnessScore() << endl;
        // cast result back to double for getPose compatibility
        return icp.getFinalTransformation().cast<double>();
    }
    cout << "[ICP] NOT converged — keeping init guess" << endl;
    return initGuess;
}

// ─────────────────────────────────────────────────────────────────────────────
// NDT alignment — same float cast pattern as ICP
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix4d ndtAlign(PointCloudT::Ptr source,
                          PointCloudT::Ptr target,
                          Eigen::Matrix4d initGuess,
                          float  resolution = 2.0f,
                          int    maxIter    = 80,
                          double stepSize   = 0.5,
                          double epsilon    = 0.001)
{
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(maxIter);
    ndt.setStepSize(stepSize);
    ndt.setTransformationEpsilon(epsilon);

    PointCloudT aligned;
    ndt.align(aligned, initGuess.cast<float>());

    if (ndt.hasConverged()){
        cout << "[NDT] converged  fitness=" << ndt.getFitnessScore()
             << "  iters=" << ndt.getFinalNumIteration() << endl;
        return ndt.getFinalTransformation().cast<double>();
    }
    cout << "[NDT] NOT converged — keeping init guess" << endl;
    return initGuess;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    auto client = cc::Client("localhost", 2000);
    client.SetTimeout(2s);
    auto world = client.GetWorld();

    auto blueprint_library = world.GetBlueprintLibrary();
    auto vehicles = blueprint_library->Filter("vehicle");

    auto map       = world.GetMap();
    auto transform = map->GetRecommendedSpawnPoints()[1];
    auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

    auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
    lidar_bp.SetAttribute("upper_fov",          "15");
    lidar_bp.SetAttribute("lower_fov",          "-25");
    lidar_bp.SetAttribute("channels",           "32");
    lidar_bp.SetAttribute("range",              "30");
    lidar_bp.SetAttribute("rotation_frequency", "60");
    lidar_bp.SetAttribute("points_per_second",  "500000");

    auto user_offset     = cg::Location(0, 0, 0);
    auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
    auto lidar_actor     = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
    auto lidar           = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
    bool new_scan = true;
    std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);

    // ── CRITICAL: initialise pose from actual spawn location, NOT (0,0,0) ────
    Pose pose(
        Point(vehicle->GetTransform().location.x,
              vehicle->GetTransform().location.y,
              vehicle->GetTransform().location.z),
        Rotate(vehicle->GetTransform().rotation.yaw   * pi/180,
               vehicle->GetTransform().rotation.pitch * pi/180,
               vehicle->GetTransform().rotation.roll  * pi/180));

    // Load map
    PointCloudT::Ptr mapCloud(new PointCloudT);
    pcl::io::loadPCDFile("map.pcd", *mapCloud);
    cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
    renderPointCloud(viewer, mapCloud, "map", Color(0,0,1));

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr scanCloud(new pcl::PointCloud<PointT>);

    lidar->Listen([&new_scan, &lastScanTime, &scanCloud](auto data){
        if (new_scan){
            auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
            for (auto detection : *scan){
                if ((detection.x*detection.x +
                     detection.y*detection.y +
                     detection.z*detection.z) > 8.0){
                    pclCloud.points.push_back(PointT(detection.x, detection.y, detection.z));
                }
            }
            if (pclCloud.points.size() > 5000){
                lastScanTime = std::chrono::system_clock::now();
                *scanCloud   = pclCloud;
                new_scan     = false;
            }
        }
    });

    Pose poseRef(
        Point(vehicle->GetTransform().location.x,
              vehicle->GetTransform().location.y,
              vehicle->GetTransform().location.z),
        Rotate(vehicle->GetTransform().rotation.yaw   * pi/180,
               vehicle->GetTransform().rotation.pitch * pi/180,
               vehicle->GetTransform().rotation.roll  * pi/180));
    double maxError = 0;

    viewer->addText("O=ICP  N=NDT  H=Hybrid(default)",
                    200, 260, 18, 0.8, 0.8, 0.0, "modeHint", 0);

    while (!viewer->wasStopped())
    {
        while (new_scan){
            std::this_thread::sleep_for(0.1s);
            world.Tick(1s);
        }

        if (refresh_view){
            viewer->setCameraPosition(
                pose.position.x, pose.position.y, 60,
                pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
            refresh_view = false;
        }

        viewer->removeShape("box0");
        viewer->removeShape("boxFill0");

        Pose truePose =
            Pose(Point(vehicle->GetTransform().location.x,
                       vehicle->GetTransform().location.y,
                       vehicle->GetTransform().location.z),
                 Rotate(vehicle->GetTransform().rotation.yaw   * pi/180,
                        vehicle->GetTransform().rotation.pitch * pi/180,
                        vehicle->GetTransform().rotation.roll  * pi/180))
            - poseRef;

        drawCar(truePose, 0, Color(1,0,0), 0.7, viewer);

        double theta  = truePose.rotation.yaw;
        double stheta = control.steer * pi/4 + theta;
        viewer->removeShape("steer");
        renderRay(viewer,
            Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta), truePose.position.z),
            Point(truePose.position.x+4*cos(stheta),truePose.position.y+4*sin(stheta),truePose.position.z),
            "steer", Color(0,1,0));

        ControlState accuate(0, 0, 1);
        if (cs.size() > 0){
            accuate = cs.back();
            cs.clear();
            Accuate(accuate, control);
            vehicle->ApplyControl(control);
        }

        viewer->spinOnce();

        if (!new_scan)
        {
            new_scan = true;

            // ── STEP 1: Voxel filter ──────────────────────────────────────────
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(scanCloud);
            vg.setLeafSize(0.5f, 0.5f, 0.5f);
            vg.filter(*cloudFiltered);
            cout << "[VoxelFilter] " << scanCloud->points.size()
                 << " -> " << cloudFiltered->points.size() << " pts" << endl;

            // ── STEP 2: Build initial guess (Matrix4d) from current pose ──────
            // Uses transform3D from helper.cpp — correct type guaranteed
            Eigen::Matrix4d initGuess = poseToMatrix(pose);

            Eigen::Matrix4d transform4x4 = Eigen::Matrix4d::Identity();

            if (locMode == 0){
                // ICP only
                transform4x4 = icpAlign(cloudFiltered, mapCloud, initGuess, 60, 4.0);
            }
            else if (locMode == 1){
                // NDT only
                transform4x4 = ndtAlign(cloudFiltered, mapCloud, initGuess, 2.0f, 80, 0.5);
            }
            else {
                // NDT + ICP hybrid (default)
                // Phase 1: NDT — coarse, robust to larger offsets
                Eigen::Matrix4d ndtResult = ndtAlign(cloudFiltered, mapCloud,
                                                     initGuess, 2.0f, 80, 0.5);
                // Phase 2: ICP — fine refinement on top of NDT result
                transform4x4 = icpAlign(cloudFiltered, mapCloud, ndtResult, 40, 2.0);
            }

            // ── getPose takes Matrix4d — confirmed from helper.h ──────────────
            pose = getPose(transform4x4);

            // ── STEP 3: Transform filtered scan into world frame ──────────────
            PointCloudT::Ptr transformedScan(new PointCloudT);
            // transformPointCloud needs float matrix — cast here
            pcl::transformPointCloud(*cloudFiltered, *transformedScan,
                                     transform4x4.cast<float>());

            viewer->removePointCloud("scan");
            renderPointCloud(viewer, transformedScan, "scan", Color(1,0,0));

            viewer->removeAllShapes();
            drawCar(pose, 1, Color(0,1,0), 0.35, viewer);

            double poseError = sqrt(
                (truePose.position.x - pose.position.x)*(truePose.position.x - pose.position.x) +
                (truePose.position.y - pose.position.y)*(truePose.position.y - pose.position.y));
            if (poseError > maxError) maxError = poseError;

            double distDriven = sqrt(
                truePose.position.x*truePose.position.x +
                truePose.position.y*truePose.position.y);

            viewer->removeShape("maxE");
            viewer->addText("Max Error: "+to_string(maxError)+" m",   200,100,32,1,1,1,"maxE",0);
            viewer->removeShape("derror");
            viewer->addText("Pose error: "+to_string(poseError)+" m", 200,150,32,1,1,1,"derror",0);
            viewer->removeShape("dist");
            viewer->addText("Distance: "+to_string(distDriven)+" m",  200,200,32,1,1,1,"dist",0);
            viewer->removeShape("modeText");
            string modeStr = (locMode==0)?"Mode: ICP":(locMode==1)?"Mode: NDT":"Mode: NDT+ICP Hybrid";
            viewer->addText(modeStr, 200,250,20,0.8,0.8,0,"modeText",0);

            if (maxError > 1.2 || distDriven >= 170.0){
                viewer->removeShape("eval");
                if (maxError > 1.2)
                    viewer->addText("Try Again",200,50,32,1,0,0,"eval",0);
                else
                    viewer->addText("Passed!",  200,50,32,0,1,0,"eval",0);
            }

            pclCloud.points.clear();
        }
    }
    return 0;
}
