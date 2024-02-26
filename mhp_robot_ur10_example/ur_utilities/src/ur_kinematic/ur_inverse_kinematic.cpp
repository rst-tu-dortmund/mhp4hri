/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund University, Institute of Control Theory and System Engineering
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
 *  Authors: Rodrigo Velasco
 *  Maintainer(s)/Modifier(s): Maximilian Krämer, Heiko Renz
 *********************************************************************/
#include <ur_utilities/ur_kinematic/ur_inverse_kinematic.h>

namespace mhp_robot {
namespace robot_kinematic {

URInverseKinematic::URInverseKinematic() : RobotInverseKinematic(std::make_unique<URKinematic>()) { parseLinkLengths(); }

void URInverseKinematic::setDesiredLastJoint(double value) { _q6_desired = value; }

void URInverseKinematic::calculateIKSolution(Eigen::MatrixXd& solutions, int& n_solutions)
{
    if (!_initialized)
    {
        ROS_ERROR("URInverseKinematic: Not yet initialized");
        return;
    }

    double T[16];
    double ik_solution[8 * 6];

    defineTransformation(_task_position, _task_orientation, T);
    n_solutions = inverse(T, ik_solution);

    if (n_solutions > 0)
    {
        // interval [-pi,pi]:
        for (int i = 0; i < (n_solutions * 6); ++i)
        {
            if (ik_solution[i] > M_PI) ik_solution[i] -= (2 * M_PI);
        }

        // store solution in matrix form
        solutions.resize(n_solutions, _n_dynamic_joints);

        for (int i = 0; i < n_solutions; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                solutions(i, j) = (ik_solution[i * 6 + j]);
            }
        }
    }
}

void URInverseKinematic::defineTransformation(const Eigen::Ref<const Eigen::Vector3d>& pos, const Eigen::Ref<const Eigen::Vector3d>& rpy, double* T)
{
    double c_roll = cos(rpy(0)), s_roll = sin(rpy(0));
    double c_pitch = cos(rpy(1)), s_pitch = sin(rpy(1));
    double c_yaw = cos(rpy(2)), s_yaw = sin(rpy(2));

    T[0]  = c_yaw * c_pitch;
    T[1]  = c_yaw * s_pitch * s_roll - s_yaw * c_roll;
    T[2]  = c_yaw * s_pitch * c_roll + s_yaw * s_roll;
    T[3]  = pos(0);
    T[4]  = s_yaw * c_pitch;
    T[5]  = s_yaw * s_pitch * s_roll + c_yaw * c_roll;
    T[6]  = s_yaw * s_pitch * c_roll - c_yaw * s_roll;
    T[7]  = pos(1);
    T[8]  = -s_pitch;
    T[9]  = c_pitch * s_roll;
    T[10] = c_pitch * c_roll;
    T[11] = pos(2);
    T[12] = 0;
    T[13] = 0;
    T[14] = 0;
    T[15] = 1;
}

int URInverseKinematic::inverse(const double* T, double* ik_solution)
{

    const double ZERO_THRESH = 0.0001;

    int num_sols = 0;
    double T02   = -*T;
    T++;
    double T00 = *T;
    T++;
    double T01 = *T;
    T++;
    double T03 = -*T;
    T++;
    double T12 = -*T;
    T++;
    double T10 = *T;
    T++;
    double T11 = *T;
    T++;
    double T13 = -*T;
    T++;
    double T22 = *T;
    T++;
    double T20 = -*T;
    T++;
    double T21 = -*T;
    T++;
    double T23 = *T;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
        double A = _d6 * T12 - T13;
        double B = _d6 * T02 - T03;
        double R = A * A + B * B;
        if (fabs(A) < ZERO_THRESH)
        {
            double div;
            if (fabs(fabs(_d4) - fabs(B)) < ZERO_THRESH)
                div = -robot_misc::Common::sgn<double>(_d4) * robot_misc::Common::sgn<double>(B);
            else
                div = -_d4 / B;
            double arcsin = asin(div);
            if (fabs(arcsin) < ZERO_THRESH) arcsin = 0.0;
            if (arcsin < 0.0)
                q1[0] = arcsin + 2.0 * M_PI;
            else
                q1[0] = arcsin;
            q1[1] = M_PI - arcsin;
        }
        else if (fabs(B) < ZERO_THRESH)
        {
            double div;
            if (fabs(fabs(_d4) - fabs(A)) < ZERO_THRESH)
                div = robot_misc::Common::sgn<double>(_d4) * robot_misc::Common::sgn<double>(A);
            else
                div = _d4 / A;
            double arccos = acos(div);
            q1[0]         = arccos;
            q1[1]         = 2.0 * M_PI - arccos;
        }
        else if (_d4 * _d4 > R)
        {
            return num_sols;
        }
        else
        {
            double arccos = acos(_d4 / sqrt(R));
            double arctan = atan2(-B, A);
            double pos    = arccos + arctan;
            double neg    = -arccos + arctan;
            if (fabs(pos) < ZERO_THRESH) pos = 0.0;
            if (fabs(neg) < ZERO_THRESH) neg = 0.0;
            if (pos >= 0.0)
                q1[0] = pos;
            else
                q1[0] = 2.0 * M_PI + pos;
            if (neg >= 0.0)
                q1[1] = neg;
            else
                q1[1] = 2.0 * M_PI + neg;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
        for (int i = 0; i < 2; i++)
        {
            double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - _d4);
            double div;
            if (fabs(fabs(numer) - fabs(_d6)) < ZERO_THRESH)
                div = robot_misc::Common::sgn<double>(numer) * robot_misc::Common::sgn<double>(_d6);
            else
                div = numer / _d6;
            double arccos = acos(div);
            q5[i][0]      = arccos;
            q5[i][1]      = 2.0 * M_PI - arccos;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                double c1 = cos(q1[i]), s1 = sin(q1[i]);
                double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
                double q6;
                ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                if (fabs(s5) < ZERO_THRESH)
                    q6 = _q6_desired;
                else
                {
                    q6 = atan2(robot_misc::Common::sgn<double>(s5) * -(T01 * s1 - T11 * c1),
                               robot_misc::Common::sgn<double>(s5) * (T00 * s1 - T10 * c1));
                    if (fabs(q6) < ZERO_THRESH) q6 = 0.0;
                    if (q6 < 0.0) q6 += 2.0 * M_PI;
                }
                ////////////////////////////////////////////////////////////////////////////////

                double q2[2], q3[2], q4[2];
                ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                double c6 = cos(q6), s6 = sin(q6);
                double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
                double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
                double p13x = _d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - _d6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1;
                double p13y = T23 - _d1 - _d6 * T22 + _d5 * (T21 * c6 + T20 * s6);

                double c3 = (p13x * p13x + p13y * p13y - _a2 * _a2 - _a3 * _a3) / (2.0 * _a2 * _a3);
                if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
                    c3 = robot_misc::Common::sgn<double>(c3);
                else if (fabs(c3) > 1.0)
                {
                    continue;
                }
                double arccos = acos(c3);
                q3[0]         = arccos;
                q3[1]         = 2.0 * M_PI - arccos;
                double denom  = _a2 * _a2 + _a3 * _a3 + 2 * _a2 * _a3 * c3;
                double s3     = sin(arccos);
                double A = (_a2 + _a3 * c3), B = _a3 * s3;
                q2[0]        = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
                q2[1]        = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
                double c23_0 = cos(q2[0] + q3[0]);
                double s23_0 = sin(q2[0] + q3[0]);
                double c23_1 = cos(q2[1] + q3[1]);
                double s23_1 = sin(q2[1] + q3[1]);
                q4[0]        = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
                q4[1]        = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
                ////////////////////////////////////////////////////////////////////////////////
                for (int k = 0; k < 2; k++)
                {
                    if (fabs(q2[k]) < ZERO_THRESH)
                        q2[k] = 0.0;
                    else if (q2[k] < 0.0)
                        q2[k] += 2.0 * M_PI;
                    if (fabs(q4[k]) < ZERO_THRESH)
                        q4[k] = 0.0;
                    else if (q4[k] < 0.0)
                        q4[k] += 2.0 * M_PI;
                    ik_solution[num_sols * 6 + 0] = q1[i];
                    ik_solution[num_sols * 6 + 1] = q2[k];
                    ik_solution[num_sols * 6 + 2] = q3[k];
                    ik_solution[num_sols * 6 + 3] = q4[k];
                    ik_solution[num_sols * 6 + 4] = q5[i][j];
                    ik_solution[num_sols * 6 + 5] = q6;
                    num_sols++;
                }
            }
        }
    }
    return num_sols;
}

void URInverseKinematic::parseLinkLengths()
{
    _d1 = _segment_structs[1].joint.tz + _segment_structs[2].joint.tz;  // pedestal_height + shoulder_height;
    _a2 = _segment_structs[4].joint.tz * -1;                            // upper_arm_length * -1;
    _a3 = _segment_structs[5].joint.tz * -1;                            // forearm_length * -1;
    _d4 = _segment_structs[3].joint.ty + _segment_structs[4].joint.ty +
          _segment_structs[6].joint.ty;  // shoulder_offset + elbow_offset + wrist1_length;
    _d5 = _segment_structs[7].joint.tz;  // wrist2_length;
    _d6 = _segment_structs[8].joint.ty;  // wrist3_length;
}

}  // namespace robot_kinematic
}  // namespace mhp_robot
