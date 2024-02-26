/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020,
 *  TU Dortmund University, Institute of Control Theory and System Enginnering
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
 *  Authors: Christoph Rösmann
 *  Modifier(s)/Maintainer(s): Maximilian Krämer, Heiko Renz
 *********************************************************************/

#ifndef SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_STAGE_FUNCTIONS_H_
#define SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_STAGE_FUNCTIONS_H_

#include <mhp_planner/core/factory.h>
#include <mhp_planner/core/reference_trajectory.h>
#include <mhp_planner/core/types.h>
#include <mhp_planner/ocp/functions/stage_preprocessor.h>

#include <mhp_planner/hypergraph/graph/generic_edge.h>

#include <Eigen/Core>

#include <memory>
#include <unordered_map>

namespace mhp_planner {

struct StageFunctionCounter
{
    static void inc(const std::string& type);
    static void setZero();
    static void print();

 private:
    static std::unordered_map<std::string, int> counts;
};

class DiscretizationGridInterface;

class StageFunction
{
 public:
    using Ptr      = std::shared_ptr<StageFunction>;
    using ConstPtr = std::shared_ptr<const StageFunction>;

    //! Default destructor
    virtual ~StageFunction() = default;

    virtual bool hasIntegralTerms(int k) const;
    virtual bool hasNonIntegralTerms(int k) const;

    virtual int getNonIntegralStateTermDimension(int k) const { return 0; }
    virtual int getNonIntegralControlTermDimension(int k) const { return 0; }
    virtual int getNonIntegralControlDeviationTermDimension(int k) const { return 0; }
    virtual int getNonIntegralDtTermDimension(int k) const { return 0; }
    virtual int getNonIntegralStateControlTermDimension(int k) const { return 0; }
    virtual int getNonIntegralStateControlDtTermDimension(int k) const { return 0; }
    virtual int getConcatenatedNonIntegralStateTermDimension(int k, bool lsq_mode = false) const;
    virtual int getConcatenatedNonIntegralStateControlTermDimension(int k, bool lsq_mode = false) const;

    virtual int getIntegralStateControlTermDimension(int k) const { return 0; }

    virtual bool isLinearNonIntegralStateTerm(int k) const { return false; }
    virtual bool isLinearNonIntegralControlTerm(int k) const { return false; }
    virtual bool isLinearNonIntegralDtTerm(int k) const { return false; }

    virtual bool isLsqFormNonIntegralStateTerm(int k) const { return false; }
    virtual bool isLsqFormNonIntegralControlTerm(int k) const { return false; }
    virtual bool isLsqFormNonIntegralDtTerm(int k) const { return false; }

    virtual bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                        bool single_dt, const Eigen::VectorXd& x0, StagePreprocessor::Ptr stage_preprocessor, const std::vector<double>& dts,
                        const DiscretizationGridInterface* grid)
    {
        return false;
    }

    virtual void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const {}
    virtual void computeNonIntegralControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, Eigen::Ref<Eigen::VectorXd> cost) const {}
    virtual void computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                        const Eigen::Ref<const Eigen::VectorXd>& u_prev, double dt_prev,
                                                        Eigen::Ref<Eigen::VectorXd> cost) const
    {
    }
    virtual void computeNonIntegralDtTerm(int k, double dt, Eigen::Ref<Eigen::VectorXd> cost) const {}
    virtual void computeNonIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                    Eigen::Ref<Eigen::VectorXd> cost) const
    {
    }
    virtual void computeNonIntegralStateControlDtTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                      const Eigen::Ref<const Eigen::VectorXd>& u_k, double dt_k,
                                                      Eigen::Ref<Eigen::VectorXd> cost) const
    {
    }

    virtual void computeIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                 Eigen::Ref<Eigen::VectorXd> cost) const
    {
    }

    // overload method for a better performance
    virtual void computeConcatenatedNonIntegralStateTerms(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                          const Eigen::Ref<const Eigen::VectorXd>& u_k, double dt_k, Eigen::Ref<Eigen::VectorXd> cost,
                                                          bool lsq_mode = false) const;

    // we exclude the control derivative term here! // overload method for a better performance
    virtual void computeConcatenatedNonIntegralStateControlTerms(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                                 const Eigen::Ref<const Eigen::VectorXd>& u_k, double dt_k,
                                                                 Eigen::Ref<Eigen::VectorXd> cost, bool lsq_mode = false) const;

    virtual bool checkParameters(int state_dim, int control_dim) const { return true; }
};

class StageCost : public StageFunction
{
 public:
    using Ptr      = std::shared_ptr<StageCost>;
    using ConstPtr = std::shared_ptr<const StageCost>;

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    // For multistage planning a planner id
    virtual void setPlannerId(const int id) {}
    virtual int getPlannerId() const { return 0; }
    virtual bool isPlannerSet() const { return true; }
    // WARNING: IntegralTerms must have a dimension of 0 or 1 !

    virtual bool fromParameterServer(const std::string& ns) = 0;
};

using StageCostFactory = Factory<StageCost>;
#define FACTORY_REGISTER_STAGE_COST(type) FACTORY_REGISTER_OBJECT(type, StageCost)

class FinalStageCost : public StageFunction
{
 public:
    using Ptr      = std::shared_ptr<FinalStageCost>;
    using ConstPtr = std::shared_ptr<const FinalStageCost>;

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    bool hasNonIntegralTerms(int k) const final { return true; }
    bool hasIntegralTerms(int k) const final { return false; }

    int getNonIntegralStateTermDimension(int k) const override = 0;

    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override = 0;

    // For multistage planning a planner id
    virtual void setPlannerId(const int id) {}
    virtual int getPlannerId() const { return 0; }
    virtual bool isPlannerSet() const { return true; }

    virtual bool fromParameterServer(const std::string& ns) = 0;

 private:
    int getNonIntegralControlTermDimension(int k) const final { return 0; }
    int getNonIntegralControlDeviationTermDimension(int k) const final { return 0; }
    int getNonIntegralDtTermDimension(int k) const final { return 0; }
    int getNonIntegralStateControlTermDimension(int k) const final { return 0; }
    int getNonIntegralStateControlDtTermDimension(int k) const final { return 0; }
    int getIntegralStateControlTermDimension(int k) const final { return 0; }
    void computeNonIntegralControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, Eigen::Ref<Eigen::VectorXd> cost) const final {}
    void computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                double dt_prev, Eigen::Ref<Eigen::VectorXd> cost) const final
    {
    }
    void computeNonIntegralDtTerm(int k, double dt, Eigen::Ref<Eigen::VectorXd> cost) const final {}
    void computeNonIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                            Eigen::Ref<Eigen::VectorXd> cost) const final
    {
    }
    void computeNonIntegralStateControlDtTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                              double dt_k, Eigen::Ref<Eigen::VectorXd> cost) const final
    {
    }

    void computeIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                         Eigen::Ref<Eigen::VectorXd> cost) const final
    {
    }
};

using FinalStageCostFactory = Factory<FinalStageCost>;
#define FACTORY_REGISTER_FINAL_STAGE_COST(type) FACTORY_REGISTER_OBJECT(type, FinalStageCost)

class FinalStageConstraint : public StageFunction
{
 public:
    using Ptr      = std::shared_ptr<FinalStageConstraint>;
    using ConstPtr = std::shared_ptr<const FinalStageConstraint>;

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    virtual bool isEqualityConstraint() const = 0;
    bool isInequalityConstraint() const { return !isEqualityConstraint(); }

    bool hasNonIntegralTerms(int k) const final { return true; }
    bool hasIntegralTerms(int k) const final { return false; }

    int getNonIntegralStateTermDimension(int k) const override = 0;

    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override = 0;

    bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                bool single_dt, const Eigen::VectorXd& x0, StagePreprocessor::Ptr stage_preprocessor, const std::vector<double>& dts,
                const DiscretizationGridInterface* grid) final
    {
        return update(n, t, xref, uref, sref, single_dt, x0, {}, stage_preprocessor, dts, grid);
    }

    virtual bool update(int n, double t, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref, ReferenceTrajectoryInterface* sref,
                        bool single_dt, const Eigen::VectorXd& x0, FinalStageCost::ConstPtr final_stage_cost,
                        StagePreprocessor::Ptr stage_preprocessor, const std::vector<double>& dts, const DiscretizationGridInterface* grid)
    {
        return false;
    }

    virtual bool checkParameters(int state_dim, int control_dim, FinalStageCost::ConstPtr final_stage_cost) const { return true; }
    // For multistage planning a planner id
    virtual void setPlannerId(const int id) {}
    virtual int getPlannerId() const { return 0; }
    virtual bool isPlannerSet() const { return true; }

    virtual bool fromParameterServer(const std::string& ns) { return true; };

 private:
    int getNonIntegralControlTermDimension(int k) const final { return 0; }
    int getNonIntegralControlDeviationTermDimension(int k) const final { return 0; }
    int getNonIntegralDtTermDimension(int k) const final { return 0; }
    int getNonIntegralStateControlTermDimension(int k) const final { return 0; }
    int getNonIntegralStateControlDtTermDimension(int k) const final { return 0; }
    int getIntegralStateControlTermDimension(int k) const final { return 0; }
    void computeNonIntegralControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, Eigen::Ref<Eigen::VectorXd> cost) const final {}
    void computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                double dt, Eigen::Ref<Eigen::VectorXd> cost) const final
    {
    }
    void computeNonIntegralDtTerm(int k, double dt, Eigen::Ref<Eigen::VectorXd> cost) const final {}
    void computeNonIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                            Eigen::Ref<Eigen::VectorXd> cost) const final
    {
    }
    void computeNonIntegralStateControlDtTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                              double dt_k_prev, Eigen::Ref<Eigen::VectorXd> cost) const final
    {
    }

    void computeIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                         Eigen::Ref<Eigen::VectorXd> cost) const final
    {
    }
};

using FinalStageConstraintFactory = Factory<FinalStageConstraint>;
#define FACTORY_REGISTER_FINAL_STAGE_CONSTRAINT(type) FACTORY_REGISTER_OBJECT(type, FinalStageConstraint)

class StageEqualityConstraint : public StageFunction
{
 public:
    using Ptr      = std::shared_ptr<StageEqualityConstraint>;
    using ConstPtr = std::shared_ptr<const StageEqualityConstraint>;

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;
};

using StageEqualitiesFactory = Factory<StageEqualityConstraint>;
#define FACTORY_REGISTER_STAGE_EQUALITIES(type) FACTORY_REGISTER_OBJECT(type, StageEqualityConstraint)

class StageInequalityConstraint : public StageFunction
{
 public:
    using Ptr      = std::shared_ptr<StageInequalityConstraint>;
    using ConstPtr = std::shared_ptr<const StageInequalityConstraint>;

    //! Return a newly created shared instance of the implemented class
    virtual Ptr getInstance() const = 0;

    // For multistage planning a planner id
    virtual void setPlannerId(const int id) {}
    virtual int getPlannerId() const { return 0; }
    virtual bool isPlannerSet() const { return true; }

    virtual bool fromParameterServer(const std::string& ns) = 0;
};

using StageInequalitiesFactory = Factory<StageInequalityConstraint>;
#define FACTORY_REGISTER_STAGE_INEQUALITIES(type) FACTORY_REGISTER_OBJECT(type, StageInequalityConstraint)

}  // namespace mhp_planner

#endif  // SRC_OPTIMAL_CONTROL_INCLUDE_MHP_PLANNER_OPTIMAL_CONTROL_FUNCTIONS_STAGE_FUNCTIONS_H_
