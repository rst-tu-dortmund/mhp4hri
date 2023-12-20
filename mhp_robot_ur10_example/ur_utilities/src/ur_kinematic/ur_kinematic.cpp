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
*  Authors: Maximilian Krämer
*  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/
#include <ur_utilities/ur_kinematic/ur_kinematic.h>

namespace mhp_robot {
namespace robot_kinematic {

URKinematic::URKinematic() : RobotKinematic(std::make_unique<robot_misc::URUtility>()) { parseLinkLengths(); }

RobotKinematic::UPtr URKinematic::createUniqueInstance() const { return std::make_unique<URKinematic>(); }

RobotKinematic::Ptr URKinematic::createSharedInstance() const { return std::make_shared<URKinematic>(); }

void URKinematic::calculateKinematicTransformations(std::vector<Eigen::Matrix<double, 4, 4>>& transformations)
{
    if (_joint_state.isApprox(_joint_state_transformations)) return;

    _joint_state_transformations = _joint_state;

    double _c1 = cos(_joint_state(0));
    double _s1 = sin(_joint_state(0));
    double _c2 = cos(_joint_state(1));
    double _s2 = sin(_joint_state(1));
    double _c3 = cos(_joint_state(2));
    double _s3 = sin(_joint_state(2));
    double _c4 = cos(_joint_state(3));
    double _s4 = sin(_joint_state(3));
    double _c5 = cos(_joint_state(4));
    double _s5 = sin(_joint_state(4));
    double _c6 = cos(_joint_state(5));
    double _s6 = sin(_joint_state(5));

    transformations[2](0, 0) = _c1;
    transformations[2](0, 1) = -_s1;
    transformations[2](1, 0) = _s1;
    transformations[2](1, 1) = _c1;

    transformations[3](0, 0) = -_s2;
    transformations[3](0, 2) = _c2;
    transformations[3](2, 0) = -_c2;
    transformations[3](2, 2) = -_s2;

    transformations[4](0, 0) = _c3;
    transformations[4](0, 2) = _s3;
    transformations[4](2, 0) = -_s3;
    transformations[4](2, 2) = _c3;

    transformations[5](0, 0) = -_s4;
    transformations[5](0, 2) = _c4;
    transformations[5](2, 0) = -_c4;
    transformations[5](2, 2) = -_s4;

    transformations[6](0, 0) = _c5;
    transformations[6](0, 1) = -_s5;
    transformations[6](1, 0) = _s5;
    transformations[6](1, 1) = _c5;

    transformations[7](0, 0) = _c6;
    transformations[7](0, 2) = _s6;
    transformations[7](2, 0) = -_s6;
    transformations[7](2, 2) = _c6;
}

void URKinematic::calculateEndEffectorMatrix(Eigen::Ref<Eigen::Matrix<double, 4, 4>> ee)
{
    if (_joint_state.isApprox(_joint_state_ee)) return;

    double qd[6];
    double T[16];

    qd[0] = _joint_state(0);
    qd[1] = _joint_state(1);
    qd[2] = _joint_state(2);
    qd[3] = _joint_state(3);
    qd[4] = _joint_state(4);
    qd[5] = _joint_state(5);

    forward(qd, T);
    ee = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(T);

    _joint_state_ee = _joint_state;
}

void URKinematic::forward(const double* q, double* T)
{
    double s1 = sin(*q), c1 = cos(*q);
    q++;
    double q234 = *q, s2 = sin(*q), c2 = cos(*q);
    q++;
    double s3 = sin(*q), c3 = cos(*q);
    q234 += *q;
    q++;
    q234 += *q;
    q++;
    double s5 = sin(*q), c5 = cos(*q);
    q++;
    double s6 = sin(*q), c6 = cos(*q);
    double s234 = sin(q234), c234 = cos(q234);

    double aux1  = c1 * c234;
    double aux2  = s1 * s234;
    double aux3  = c1 * s234;
    double aux4  = s1 * c234;
    double aux5  = aux1 + aux2;
    double aux6  = aux1 - aux2;
    double aux7  = aux4 + aux3;
    double aux8  = aux4 - aux3;
    double aux9  = s1 * s5 + (aux6 * c5) / 2.0 + (aux5 * c5) / 2.0;
    double aux10 = (aux7 * c5) / 2.0 - c1 * s5 + (aux8 * c5) / 2.0;
    double aux11 = c234 * c5;
    double aux12 = s234 * s5;
    double aux13 = s234 * c6;
    double aux14 = c234 * s6;
    double aux15 = c234 * c6;
    double aux16 = s234 * s6;
    double aux17 = aux11 - aux12;
    double aux18 = aux11 + aux12;
    double aux19 = aux13 + aux14;
    double aux20 = aux13 - aux14;
    double aux21 = aux15 + aux16;
    double aux22 = aux15 - aux16;
    double aux23 = (aux6 * s5) / 2.0 - c5 * s1 + (aux5 * s5) / 2.0;
    double aux24 = (c6 * aux9 - (s6 * (aux7 - aux8)) / 2.0);
    double aux25 = (-(c6 * (aux7 - aux8)) / 2.0 - s6 * aux9);
    double aux26 = ((_d5 * aux8) / 2.0 - (_d5 * aux7) / 2.0 - _d4 * s1 + (_d6 * aux6 * s5) / 2.0 + (_d6 * aux5 * s5) / 2.0 - _a2 * c1 * c2 -
                    _d6 * c5 * s1 - _a3 * c1 * c2 * c3 + _a3 * c1 * s2 * s3);
    double aux27 = c1 * c5 + (aux7 * s5) / 2.0 + (aux8 * s5) / 2.0;
    double aux28 = (c6 * aux10 + s6 * (aux6 / 2.0 - aux5 / 2.0));
    double aux29 = (c6 * (aux6 / 2.0 - aux5 / 2.0) - s6 * aux10);
    double aux30 = ((_d5 * aux6) / 2.0 - (_d5 * aux5) / 2.0 + _d4 * c1 + (_d6 * aux7 * s5) / 2.0 + (_d6 * aux8 * s5) / 2.0 + _d6 * c1 * c5 -
                    _a2 * c2 * s1 - _a3 * c2 * c3 * s1 + _a3 * s1 * s2 * s3);

    *T = (_c0 * aux23) - (_s0 * aux27);
    T++;
    *T = (_c0 * aux24) - (_s0 * aux28);
    T++;
    *T = (_c0 * aux25) - (_s0 * aux29);
    T++;
    *T = (_c0 * aux26) - (_s0 * aux30) + _dx;
    T++;
    *T = (_s0 * aux23) + (_c0 * aux27);
    T++;
    *T = (_s0 * aux24) + (_c0 * aux28);
    T++;
    *T = (_s0 * aux25) + (_c0 * aux29);
    T++;
    *T = (_s0 * aux26) + (_c0 * aux30) + _dy;
    T++;
    *T = (aux17 / 2.0 - aux18 / 2.0);
    T++;
    *T = (aux20 / 2.0 - aux19 / 2.0 - s234 * c5 * c6);
    T++;
    *T = (s234 * c5 * s6 - aux21 / 2.0 - aux22 / 2.0);
    T++;
    *T = (_d1 + (_d6 * aux17) / 2.0 + _a3 * (s2 * c3 + c2 * s3) + _a2 * s2 - (_d6 * aux18) / 2.0 - _d5 * c234) + _dz;
    T++;
    *T = 0.0;
    T++;
    *T = 0.0;
    T++;
    *T = 0.0;
    T++;
    *T = 1.0;
}

void URKinematic::parseLinkLengths()
{
    _d1 = _segment_structs[1].joint.tz + _segment_structs[2].joint.tz;  // pedestal_height + shoulder_height;
    _a2 = _segment_structs[4].joint.tz * -1;                            // upper_arm_length * -1;
    _a3 = _segment_structs[5].joint.tz * -1;                            // forearm_length * -1;
    _d4 = _segment_structs[3].joint.ty + _segment_structs[4].joint.ty +
          _segment_structs[6].joint.ty;  // shoulder_offset + elbow_offset + wrist1_length;
    _d5 = _segment_structs[7].joint.tz;  // wrist2_length;
    _d6 = _segment_structs[8].joint.ty;  // wrist3_length;

    _c0 = _transformations[0](0, 0);  // cos angle of robot base relative to world
    _s0 = _transformations[0](1, 0);  // sin angle of robot base relative to world
    _dx = _transformations[0](0, 3);  // x position of robot base
    _dy = _transformations[0](1, 3);  // y position of robot base
    _dz = _transformations[0](2, 3);  // z position of robot base
}

}  // namespace robot_kinematic
}  // namespace mhp_robot
