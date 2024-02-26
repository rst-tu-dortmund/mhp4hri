/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Heiko Renz
 *********************************************************************/

#include <assert.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <Eigen/Geometry>
#include <map>
#include <tuple>
// Macro source: https://stackoverflow.com/questions/14038589/what-is-the-canonical-way-to-check-for-errors-using-the-cuda-runtime-api
#define gpuErrchk(ans)                        \
    {                                         \
        gpuAssert((ans), __FILE__, __LINE__); \
    }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true)
{

    if (code != cudaSuccess)
    {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) exit(code);
    }
}

// Node definitions from Ufomap source
using INNER_NODE = ufo::map::OccupancyMapInnerNode<ufo::map::ColorOccupancyNode<float>>;
using LEAF_NODE  = ufo::map::OccupancyMapLeafNode<ufo::map::ColorOccupancyNode<float>>;
using DepthType  = unsigned int;

// Global function means it will be executed on the device (GPU)
// Calculates the information gain for rays to various startpoints
__global__ void calcInfoGain(double* out, ufo::map::OccupancyMapColor* map, ufo::map::Point3* origin, ufo::map::Point3* endpoints, double* max_range,
                             int* num_endpoints, int* num_startpoints)
{
    // Get the index of the current thread
    int index_x = threadIdx.x + blockIdx.x * blockDim.x;
    int index_y = threadIdx.y + blockIdx.y * blockDim.y;
    if ((index_x < num_endpoints[0]) && (index_y < num_startpoints[0]))  // if index outside of range, do nothing to avoid memory access errors
    {
        int index = index_x + index_y * num_endpoints[0];

        // Check if the endpoint and startpoint is within the map
        double min_allowed = map->getMin()[0];
        double max_allowed = map->getMax()[0];

        assert((min_allowed <= origin[blockIdx.y].min() && max_allowed >= origin[blockIdx.y].max() && min_allowed <= endpoints[index].min() &&
                max_allowed >= endpoints[index].max()));

        // Calculate the direction and distance of the ray
        ufo::map::Point3 direction = (endpoints[index] - origin[blockIdx.y]);
        double distance            = direction.norm();
        direction /= distance;

        // Check if the ray is within the max range
        if (0 <= max_range[0] && distance > max_range[0])
        {
            endpoints[index] = origin[blockIdx.y] + (direction * max_range[0]);
            distance         = max_range[0];
        }

        // Define variables for the raycasting
        ufo::map::Key current;
        ufo::map::Key ending;
        std::array<int, 3> step;
        ufo::map::Point3 t_delta;
        ufo::map::Point3 t_max;
        double info_val = 0;

        // Compute the raycasting
        map->computeRayInit(origin[blockIdx.y], endpoints[index], direction, current, ending, step, t_delta, t_max);
        // Increment
        int i         = 0;
        bool occupied = false;
        while ((current.getDepth() != ending.getDepth() || !current.equals(ending)) && t_max.min() <= distance && !occupied)
        {
            i++;
            // Compute the information gain for the current node
            ufo::map::OccupancyState state = map->getState(current);
            switch (state)
            {
                case ufo::map::OccupancyState::occupied:  // If the node is occupied, we
                                                          // want to know less about it
                                                          // --> Onyl small increae
                    info_val = info_val + (1 - map->getOccupancy(current));
                    occupied = true;
                    break;
                case ufo::map::OccupancyState::free:  // If the node is free,  we
                                                      // want to know less about it
                                                      // --> Onyl small increae
                    info_val = info_val + (map->getOccupancy(current));

                    break;
                case ufo::map::OccupancyState::unknown:  // If the node is unknown -->
                                                         // We want to know more
                    info_val++;
                    break;
            }
            // Increment the ray
            map->computeRayTakeStep(current, step, t_delta, t_max);
        }
        // Write the information gain to the output array
        out[index] = info_val / i;
    }
}

// This function is used to get the children pointer for all nodes from the host to the device
void getChildrenPointer(std::vector<void*>* children_h, INNER_NODE* root, ufo::map::OccupancyMapColor* map_h, int depth_cnt = 0, int depth_max = 16)
{

    // Initialize variables
    void* d_children;
    INNER_NODE& inner_node = *root;

    // Recursion stop condition if we reached the max depth
    if (depth_cnt == depth_max - 1) return;

    // Check if the current node is a leaf node and return
    if (!map_h->hasChildren(inner_node))
    {
        return;
    }
    else  // If the current node is an inner node, we want to get the children pointer
    {
        // Get the children pointer from the host (recursive call)
        for (int i = 0; i <= 7; i++)
        {
            INNER_NODE& child_node = static_cast<INNER_NODE&>(map_h->getChild(inner_node, 1, i));
            if (!child_node.is_leaf && child_node.children != nullptr) getChildrenPointer(children_h, &child_node, map_h, depth_cnt + 1);
        }
        // Push the children pointer to the vector
        children_h->push_back(inner_node.children);
        // Allocate memory on the device and copy the children pointer to the device
        gpuErrchk(cudaMalloc((void**)&d_children, sizeof(INNER_NODE) * 8));
        gpuErrchk(cudaMemcpy(d_children, inner_node.children, sizeof(INNER_NODE) * 8, cudaMemcpyHostToDevice));
        // Set the children pointer of the current node to the device pointer for copying the whole map with device pointers
        root->children = d_children;
        return;
    }
    // }
}

// This function is used to set the children pointer back to the host pointers
void setPointerBackToHost(std::vector<void*>* children_h, INNER_NODE* root, ufo::map::OccupancyMapColor* map_h, int depth_cnt = 0, int depth_max = 16)
{
    // Initialize variables
    void* h_children       = (void*)malloc(sizeof(INNER_NODE) * 8);
    INNER_NODE& inner_node = *root;

    // Recursion stop condition if we reached the max depth
    if (depth_cnt == depth_max - 1) return;

    // Check if the current node is a leaf node and return
    if (!map_h->hasChildren(inner_node))
    {
        return;
    }
    else  // If the current node is an inner node, we want to get the children pointer back to the host
    {
        // Get the children pointer from the device
        gpuErrchk(cudaMemcpy(h_children, inner_node.children, sizeof(INNER_NODE) * 8, cudaMemcpyDeviceToHost));
        // Set the children pointer of the current node to the host pointer
        root->children = h_children;
        // Get the children pointer from the device and set to host pointer (recursive call)
        for (int i = 0; i <= 7; i++)
        {
            INNER_NODE& child_node = static_cast<INNER_NODE&>(map_h->getChild(inner_node, 1, i));
            if (!child_node.is_leaf && child_node.children != nullptr) setPointerBackToHost(children_h, &child_node, map_h, depth_cnt + 1);
        }
    }
}

void calcInfoGainGPU(int num, int threads, ufo::map::OccupancyMapColor* map,
                     std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>>* start_points,
                     std::map<int, std::vector<ufo::map::Point3>>* endpoints, double max_range, Eigen::MatrixXd* results,
                     Eigen::Vector4d* time_metrics)
{
    std::chrono::time_point t1 = std::chrono::steady_clock::now();

    // Initialize variables
    ufo::map::OccupancyMapColor map_h(*map);

    // host copy of output
    double* out;
    std::vector<ufo::map::Point3> endpoints_h(start_points->size() * num);
    std::vector<ufo::map::Point3> startpoints_h(start_points->size());
    for (int i = 0; i < start_points->size(); i++)
    {
        startpoints_h[i]                  = std::get<1>(start_points->at(i));
        std::vector<ufo::map::Point3>* ep = &endpoints->at(std::get<0>(start_points->at(i)));
        for (int j = 0; j < num; j++)
        {
            endpoints_h[i * num + j] = ep->at(j);
        }
    }
    int num_startpoints = startpoints_h.size();

    // device copies of inputs and output
    double* d_out;
    int *d_num_endpoints, *d_num_startpoints;
    double* d_max_range;
    ufo::map::OccupancyMapColor* d_map;
    ufo::map::Point3* d_endpoints;
    ufo::map::Point3* d_start;

    // Get the children pointer from the host to the device
    std::vector<void*> children_h;
    children_h.clear();
    int depth_max    = map_h.getTreeDepthLevels();
    INNER_NODE* node = &map_h.getRoot();
    getChildrenPointer(&children_h, node, &map_h, 0, depth_max);

    // Alloc space for device copies
    gpuErrchk(cudaMalloc((void**)&d_out, sizeof(double) * num * num_startpoints));                  // one double for each endpoint
    gpuErrchk(cudaMalloc((void**)&d_map, sizeof(ufo::map::OccupancyMapColor)));                     // alloc map space
    gpuErrchk(cudaMalloc((void**)&d_endpoints, sizeof(ufo::map::Point3) * num * num_startpoints));  // alloc space for endpoints
    gpuErrchk(cudaMalloc(
        (void**)&d_start,
        sizeof(ufo::map::Point3) *
            num_startpoints));  // alloc space for startpoint (only one); TODO(renz): Check if parallelization for startpoints is also possible
    gpuErrchk(cudaMalloc((void**)&d_max_range, sizeof(double)));     // alloc space for max_range double
    gpuErrchk(cudaMalloc((void**)&d_num_endpoints, sizeof(int)));    // alloc space for num_endpoints int
    gpuErrchk(cudaMalloc((void**)&d_num_startpoints, sizeof(int)));  // alloc space for num_startpoints int

    // Alloc space for host copies of info gain for all endpoints
    out = (double*)malloc(sizeof(double) * num * num_startpoints);  // one double for each endpoint

    // Copy data to device (Note that the node pointers for children are already copied to the device)
    gpuErrchk(cudaMemcpy(d_map, &map_h, sizeof(ufo::map::OccupancyMapColor),
                         cudaMemcpyHostToDevice));  // copy map to device (with new device pointers for children)

    gpuErrchk(cudaMemcpy(d_max_range, &max_range, sizeof(double), cudaMemcpyHostToDevice));           // copy max_range to device
    gpuErrchk(cudaMemcpy(d_num_endpoints, &num, sizeof(int), cudaMemcpyHostToDevice));                // copy num_endpoints to device
    gpuErrchk(cudaMemcpy(d_num_startpoints, &num_startpoints, sizeof(int), cudaMemcpyHostToDevice));  // copy num_endpoints to device
 

    gpuErrchk(cudaMemcpy(d_start, startpoints_h.data(), sizeof(ufo::map::Point3) * num_startpoints,
                         cudaMemcpyHostToDevice));  // copy startpoint to device
    gpuErrchk(cudaMemcpy(d_endpoints, endpoints_h.data(), sizeof(ufo::map::Point3) * num * num_startpoints,
                         cudaMemcpyHostToDevice));  // copy endpoints to device
    gpuErrchk(cudaPeekAtLastError());  // Error check

    // Launch kernel
    dim3 gridDim(static_cast<int>(std::ceil(static_cast<double>(num) / static_cast<double>(threads))), num_startpoints);

    std::chrono::time_point t2 = std::chrono::steady_clock::now();

    calcInfoGain<<<gridDim, threads>>>(d_out, d_map, d_start, d_endpoints, d_max_range, d_num_endpoints, d_num_startpoints);
    gpuErrchk(cudaDeviceSynchronize());

    std::chrono::time_point t3 = std::chrono::steady_clock::now();

    // Copy result back to host
    gpuErrchk(cudaMemcpy(out, d_out, sizeof(double) * num * num_startpoints, cudaMemcpyDeviceToHost));

    for (int blockIdx = 0; blockIdx < num_startpoints; blockIdx++)
    {
        results->row(blockIdx) = Eigen::Map<Eigen::VectorXd>(out + blockIdx * num, num);
    }

    // Copy result back to host
    setPointerBackToHost(&children_h, node, &map_h);  // Required since the destructor needs the correct host pointers to free the memory on the host

    free(out);

    // Cleanup on device
    cudaFree(d_out);
    cudaFree(d_map);
    cudaFree(d_endpoints);
    cudaFree(d_start);
    cudaFree(d_max_range);
    cudaFree(d_num_endpoints);
    cudaFree(d_num_startpoints);

    std::chrono::time_point t4 = std::chrono::steady_clock::now();

    *time_metrics = Eigen::Vector4d{std::chrono::duration<float, std::chrono::seconds::period>(t2 - t1).count(),
                                     std::chrono::duration<float, std::chrono::seconds::period>(t3 - t2).count(),
                                     std::chrono::duration<float, std::chrono::seconds::period>(t4 - t3).count(),
                                     std::chrono::duration<float, std::chrono::seconds::period>(t2 - t1).count() +
                                         std::chrono::duration<float, std::chrono::seconds::period>(t4 - t3).count()};

}
