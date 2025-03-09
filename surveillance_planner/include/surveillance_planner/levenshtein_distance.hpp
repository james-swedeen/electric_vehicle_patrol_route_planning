/**
 * @File: levenshtein_distance.hpp
 * @Date: June 2024
 * @Author: James Swedeen
 *
 * @brief
 * Defines the difference between two plans in terms of the number of hotspot visits that must be
 * inserted, deleted, or substituted to tern one plan into another.
 *
 * @cite:
 * Boytsov, Leonid. "Indexing methods for approximate dictionary searching: Comparative analysis." Journal of Experimental Algorithmics (JEA) 16 (2011): 1-1.
 **/

#ifndef SURVEILLANCE_PLANNING_LEVENSHTEIN_DISTANCE_HPP
#define SURVEILLANCE_PLANNING_LEVENSHTEIN_DISTANCE_HPP

/* C++ Headers */
#include<limits>
#include<memory>
#include<chrono>
#include<vector>
#include<deque>
#include<map>
#include<numeric>
#include<execution>
#include<algorithm>
#include<iostream> // TODO

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<surveillance_planner/action.hpp>
#include<surveillance_planner/helpers.hpp>

namespace plan
{
namespace ld
{
/**
 * @EditOp
 *
 * @brief
 * Defines a basic edit operation and where in two strings it is to be used.
 *
 * @templates
 * EIG_OPTIONS: Eigen storage options
 **/
template<Eigen::StorageOptions EIG_OPTIONS>
class EditOp
{
public:
  /**
   * @Type
   *
   * @brief
   * Defines what type of edit operation this is.
   **/
  enum class Type : uint8_t
  {
    /**
     * @NULL_TYPE
     *
     * @brief
     * This explicitly means nothing.
     **/
    NULL_TYPE = 0,
    /**
     * @Substitution
     *
     * @brief
     * Substitute one action of the start plan with another action.
     **/
    Substitution = 1,
    /**
     * @Insertion
     *
     * @brief
     * Insert an action in the given location of the start plan.
     **/
    Insertion = 2,
    /**
     * @Deletion
     *
     * @brief
     * Remove an action in the given location of the start plan.
     **/
    Deletion = 3
    /**
     * @Transposition
     *
     * @brief
     * Transform a, u_1, u_2, ..., u_nu, b into b, v_1, v_2, ..., v_nv, a.
     **/
//    Transposition = 4
  };
  /**
   * @To Stream Operator
   **/
  friend inline std::ostream& operator<<(std::ostream& os, const Type& type) noexcept
  {
    switch(type)
    {
      case EditOp::Type::NULL_TYPE:
        os << "Null";
        break;
      case EditOp::Type::Substitution:
        os << "Substitution";
        break;
      case EditOp::Type::Insertion:
        os << "Insertion\t";
        break;
      case EditOp::Type::Deletion:
        os << "Deletion\t";
        break;
//      case EditOp::Type::Transposition:
//        os << "Transposition";
//        break;
      default:
        assert(false);
        break;
    }
    return os;
  }
  /**
   * @Cost Functions
   *
   * @brief
   * Defines the cost of this edit operation.
   *
   * @parameters
   * old_action: The action before the operation (needed for substitutions only)
   * new_action: The action after the operation (needed for substitutions only)
   * max_dwell_time: The max dwell time possible in minutes (needed for substitutions only)
   * old_num_middle_actions: The number of actions between the two actions to be swapped in the original plan (needed for transpositions only)
   * new_num_middle_actions: The number of actions between the two actions to be swapped in the plan after the operation (needed for transpositions only)
   *
   * @return
   * The cost of the operation.
   **/
  static           inline uint32_t substitutionCost(const Action& old_action, const Action& new_action, const uint32_t max_dwell_time) noexcept;
  static constexpr inline uint32_t insertionCost()                                                                                   noexcept;
  static constexpr inline uint32_t deletionCost()                                                                                    noexcept;
  static           inline uint32_t transpositionCost(const uint32_t old_num_middle_actions, const uint32_t new_num_middle_actions)       noexcept;

  static constexpr const uint32_t BASE_COST_VERTEX = 4096; // The base cost of a vertex differing for the cost calculations
  static constexpr const uint32_t BASE_COST_PATH   = 2048; // The base cost of a path differing for the cost calculations
  static constexpr const uint32_t BASE_COST_TIME   = 1024; // The base cost of a dwell time differing for the cost calculations

  /// Plan relation variables
  uint32_t effected_ind;      // The flattened start plan index of the action that is the first to be modified by this operation
  uint32_t corresponding_ind; // The flattened target plan index of the action that is the first to be modified by this operation
  /// Defining variables
  uint32_t cost;       // The cost of this edit operation
  Action   old_action; // For transposition, substitution, and deletion: The old actions that get removed between the swapped actions and the swapped actions, in order
  Action   new_action; // For transposition, substitution, and insertion: The new actions that get inserted in the middle of the swapped actions
  Type     type;       // The type of edit operation this class is

  inline EditOp()                         noexcept;
  inline EditOp(const EditOp&)            noexcept = default;
  inline EditOp(EditOp&&)                 noexcept = default;
  inline EditOp& operator=(const EditOp&) noexcept = default;
  inline EditOp& operator=(EditOp&&)      noexcept = default;
  inline ~EditOp()                        noexcept = default;

  /**
   * @Constructor
   *
   * @brief
   * This constructor initializes the class for use.
   **/
  inline EditOp(const uint32_t cost,
                const uint32_t effected_ind,
                const uint32_t corresponding_ind,
                const Action&  old_action,
                const Action&  new_action) noexcept; // For substitution
  inline EditOp(const uint32_t cost,
                const uint32_t effected_ind,
                const uint32_t corresponding_ind,
                const Action&  new_action) noexcept; // For insertion
  inline EditOp(const uint32_t cost,
                const Action&  old_action,
                const uint32_t effected_ind,
                const uint32_t corresponding_ind) noexcept; // For deletion
  /**
   * @To Stream Operator
   **/
  friend inline std::ostream& operator<<(std::ostream& os, const EditOp& op) noexcept
  {
    os << "eff_ind: " << op.effected_ind
       << "\tcorr_ind: " << op.corresponding_ind
       << "\ttype: " << op.type
       << "\tcost: " << op.cost
       << "\told_ac: " << op.old_action
       << "\tnew_ac: " << op.new_action;
    return os;
  }
};
// Typedefs
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS = Eigen::StorageOptions(Eigen::ColMajor bitor Eigen::AutoAlign)>
//using ALIGNMENT_TYPE = Eigen::Matrix<std::shared_ptr<EditOp<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,1>;
using ALIGNMENT_TYPE = Eigen::Matrix<EditOp<EIG_OPTIONS>,Eigen::Dynamic,1,EIG_OPTIONS,(MAX_PLAN_VISITS == Eigen::Dynamic) ? Eigen::Dynamic : 2*MAX_PLAN_VISITS,1>;
/**
 * @To Stream Operator
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
inline std::ostream& printAlignment(std::ostream& os, const Eigen::Ref<const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& alignment) noexcept;
/**
 * @findDistance
 *
 * @brief
 * Finds the Levenshtein distance between the start and target plans.
 *
 * @parameters
 * start_plan: The plan to convert into the target plan
 * target_plan: The plan to make the start plan into
 * max_dwell_time: The max dwell time possible in minutes
 * memory_buff: A memory buffer to avoid dynamic memory allocation
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * first: The distance value
 * second: The alignment from start_plan to target_plan
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::pair<uint32_t,std::unique_ptr<ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>>
  findDistance(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& start_plan,
               const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& target_plan,
               const uint32_t                                max_dwell_time) noexcept;
/**
 * @reverseAlignment
 *
 * @brief
 * Converts an alignment from start to target plans to an alignment from target to start plans.
 *
 * @parameters
 * alignment: The start to target alignment
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * The alignment from target to start plans.
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::unique_ptr<ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  reverseAlignment(const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& alignment) noexcept;
/**
 * @applyEditOp
 *
 * @brief
 * Applies the given edit operation to the given plan.
 *
 * @parameters
 * edit_op: The edit operation to apply
 * plan: The plan to apply the operation to
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * The new plan.
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  applyEditOp(const EditOp<EIG_OPTIONS>& edit_op, const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& plan) noexcept;
/**
 * @applyAlignment
 *
 * @brief
 * Applies the given list of edit operations to the given plan.
 *
 * @parameters
 * alignment: The edit operations to apply
 * plan: The plan to apply the operation to
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * The new plan.
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  applyAlignment(const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& alignment,
                 const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&      plan) noexcept;
/**
 * @performMovement
 *
 * @brief
 * Applies a random selection of movement operators whose cost does not exceed the given value.
 *
 * @parameters
 * alignment: The edit operations to apply
 * plan: The flat plan to apply the operators to
 * max_move_cost: The max summed cost allowed from each edit operation
 * rand_gen: The random number generator to use
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * First: The new plan.
 * Second: True iff a new plan has been made.
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,bool>
  performMovement(const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& alignment,
                  const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&      plan,
                  const double                                       max_move_cost,
                  RAND_GEN_TYPE&                                     rand_gen) noexcept;
/**
 * @findIntermediateSolutions
 *
 * @brief
 * Applies the movement operators in a random order and returns all plans that satisfy plan form constraints.
 *
 * @parameters
 * alignment: The edit operations to apply
 * plan: The flat plan to apply the operators to
 * rand_gen: The random number generator to use
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * Each intermediate plan.
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::vector<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>>
  findIntermediateSolutions(const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& alignment,
                            const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&      plan,
                            RAND_GEN_TYPE&                                     rand_gen) noexcept;
} // ld


template<Eigen::StorageOptions EIG_OPTIONS>
inline uint32_t ld::EditOp<EIG_OPTIONS>::substitutionCost(const Action& old_action, const Action& new_action, const uint32_t max_dwell_time) noexcept
{
  return (BASE_COST_VERTEX * uint32_t(old_action.vertex_ind != new_action.vertex_ind)) +
         (BASE_COST_PATH   * uint32_t(old_action.prev_path  != new_action.prev_path)) +
         (BASE_COST_TIME   * ((uint32_t(std::abs(int64_t(old_action.dwell_time) - int64_t(new_action.dwell_time))) + max_dwell_time - 1) / max_dwell_time));
}

template<Eigen::StorageOptions EIG_OPTIONS>
constexpr inline uint32_t ld::EditOp<EIG_OPTIONS>::insertionCost() noexcept
{
  return BASE_COST_VERTEX + BASE_COST_PATH + BASE_COST_TIME;
}

template<Eigen::StorageOptions EIG_OPTIONS>
constexpr inline uint32_t ld::EditOp<EIG_OPTIONS>::deletionCost() noexcept
{
  return BASE_COST_VERTEX + BASE_COST_PATH + BASE_COST_TIME;
}

template<Eigen::StorageOptions EIG_OPTIONS>
inline uint32_t ld::EditOp<EIG_OPTIONS>::transpositionCost(const uint32_t old_num_middle_actions, const uint32_t new_num_middle_actions) noexcept
{
  return (BASE_COST_VERTEX + BASE_COST_PATH + BASE_COST_TIME) * (old_num_middle_actions + new_num_middle_actions + 1);
}

template<Eigen::StorageOptions EIG_OPTIONS>
inline ld::EditOp<EIG_OPTIONS>::EditOp() noexcept
 : type(Type::NULL_TYPE)
{}

template<Eigen::StorageOptions EIG_OPTIONS>
inline ld::EditOp<EIG_OPTIONS>::EditOp(const uint32_t cost,
                                       const uint32_t effected_ind,
                                       const uint32_t corresponding_ind,
                                       const Action&  old_action,
                                       const Action&  new_action) noexcept
 : effected_ind(effected_ind),
   corresponding_ind(corresponding_ind),
   cost(cost),
   old_action(old_action),
   new_action(new_action),
   type(Type::Substitution)
{
  assert(cost >= 0);
}

template<Eigen::StorageOptions EIG_OPTIONS>
inline ld::EditOp<EIG_OPTIONS>::EditOp(const uint32_t cost,
                                       const uint32_t effected_ind,
                                       const uint32_t corresponding_ind,
                                       const Action&  new_action) noexcept
 : effected_ind(effected_ind),
   corresponding_ind(corresponding_ind),
   cost(cost),
   new_action(new_action),
   type(Type::Insertion)
{
  assert(cost >= 0);
}

template<Eigen::StorageOptions EIG_OPTIONS>
inline ld::EditOp<EIG_OPTIONS>::EditOp(const uint32_t cost,
                                       const Action&  old_action,
                                       const uint32_t effected_ind,
                                       const uint32_t corresponding_ind) noexcept
 : effected_ind(effected_ind),
   corresponding_ind(corresponding_ind),
   cost(cost),
   old_action(old_action),
   type(Type::Deletion)
{
  assert(cost >= 0);
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::ostream& ld::printAlignment(std::ostream& os, const Eigen::Ref<const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& alignment) noexcept
{
  const Eigen::Index len = alignment.size();
  for(Eigen::Index ind = 0; ind < len; ++ind)
  {
    os << alignment[ind] << "\n";
  }
  return os;
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::pair<uint32_t,std::unique_ptr<ld::ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>>
  ld::findDistance(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& start_plan,
                        const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& target_plan,
                        const uint32_t                                max_dwell_time) noexcept
{
  /// Initialize helper variables
  const uint32_t len_start  = start_plan. size();
  const uint32_t len_target = target_plan.size();

  Eigen::Matrix<typename EditOp<EIG_OPTIONS>::Type,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS,(Eigen::Dynamic == MAX_PLAN_VISITS) ? Eigen::Dynamic : MAX_PLAN_VISITS+1,(Eigen::Dynamic == MAX_PLAN_VISITS) ? Eigen::Dynamic : MAX_PLAN_VISITS+1> ops_used(len_start + 1, len_target + 1);

  Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS,(Eigen::Dynamic == MAX_PLAN_VISITS) ? Eigen::Dynamic : MAX_PLAN_VISITS+1> cur_cost( len_target + 1);
  Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS,(Eigen::Dynamic == MAX_PLAN_VISITS) ? Eigen::Dynamic : MAX_PLAN_VISITS+1> next_cost(len_target + 1);

  for(uint32_t target_ind = 0; target_ind < (len_target+1); ++target_ind)
  {
    cur_cost[target_ind]    = EditOp<EIG_OPTIONS>::insertionCost() * target_ind;
    ops_used(0, target_ind) = EditOp<EIG_OPTIONS>::Type::Insertion;
  }

  // Perform recursion
  for(uint32_t start_ind = 0; start_ind < len_start; ++start_ind)
  {
    next_cost[0]               = EditOp<EIG_OPTIONS>::deletionCost() * (start_ind + 1);
    ops_used(start_ind + 1, 0) = EditOp<EIG_OPTIONS>::Type::Deletion;
    for(uint32_t target_ind = 0; target_ind < len_target; ++target_ind)
    {
      if(start_plan[start_ind] == target_plan[target_ind])
      {
        next_cost[target_ind + 1] = cur_cost[target_ind];
        ops_used(start_ind + 1, target_ind + 1) = EditOp<EIG_OPTIONS>::Type::NULL_TYPE;
        continue;
      }
      // Make possible edit operations
      Eigen::Matrix<uint32_t,3,1,EIG_OPTIONS> potential_costs;
      // Substitution
      const uint32_t substitution_sub_cost = EditOp<EIG_OPTIONS>::substitutionCost(start_plan[start_ind], target_plan[target_ind], max_dwell_time);
      potential_costs[0] = cur_cost[target_ind] + substitution_sub_cost;
      // Insertion
      potential_costs[1] = next_cost[target_ind] + EditOp<EIG_OPTIONS>::insertionCost();
      // Deletion
      potential_costs[2] = cur_cost[target_ind + 1] + EditOp<EIG_OPTIONS>::deletionCost();
      // Find min
      Eigen::Index min_op_ind;
      potential_costs.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&min_op_ind);
      // Make edit operation object
      ops_used(start_ind + 1, target_ind + 1) = typename EditOp<EIG_OPTIONS>::Type(min_op_ind + 1);
      // Update cost matrix
      next_cost[target_ind + 1] = potential_costs[min_op_ind];
    }
    std::swap(next_cost, cur_cost);
  }

  // Convert to Eigen
  std::unique_ptr<ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output     = std::make_unique<ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(len_start + len_target);
  Eigen::Index                                                 output_ind = 0;

  Eigen::Index ind0 = len_start;
  Eigen::Index ind1 = len_target;
  while(true) // (nullptr != memory_buff.second(ind0, ind1).get())
  {
    bool go_flag = (ind0 > 0) or (ind1 > 0);
    while(go_flag and (EditOp<EIG_OPTIONS>::Type::NULL_TYPE == ops_used(ind0, ind1)))
    {
      --ind0;
      --ind1;
      go_flag = (ind0 > 0) or (ind1 > 0);
    }
    if(not go_flag) { break; }

    switch(ops_used(ind0, ind1))
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        (*output)[output_ind] = EditOp<EIG_OPTIONS>(EditOp<EIG_OPTIONS>::substitutionCost(start_plan[ind0-1], target_plan[ind1-1], max_dwell_time),
                                                    ind0-1,
                                                    ind1-1,
                                                    start_plan[ind0-1],
                                                    target_plan[ind1-1]);
        --ind0;
        --ind1;
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
        (*output)[output_ind] = EditOp<EIG_OPTIONS>(EditOp<EIG_OPTIONS>::insertionCost(),
                                                    ind0,
                                                    ind1-1,
                                                    target_plan[ind1-1]);
        --ind1;
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        (*output)[output_ind] = EditOp<EIG_OPTIONS>(EditOp<EIG_OPTIONS>::deletionCost(),
                                                    start_plan[ind0-1],
                                                    ind0-1,
                                                    ind1-1);
        --ind0;
        break;
      default:
        assert(false);
        break;
    };
    ++output_ind;
  }
  if(output->size() != output_ind)
  {
    output->conservativeResize(output_ind, Eigen::NoChange);
  }
  assert(std::is_sorted(output->cbegin(), output->cend(),
                        [] (const ld::EditOp<EIG_OPTIONS>& lhs, const ld::EditOp<EIG_OPTIONS>& rhs) -> bool
                        {
                          return  (lhs.effected_ind >  rhs.effected_ind) or
                                 ((lhs.effected_ind == rhs.effected_ind) and (lhs.corresponding_ind > rhs.corresponding_ind));
                        }));
  assert(cur_cost.template bottomRows<1>()[0] == output->unaryExpr([] (const EditOp<EIG_OPTIONS>& op) -> uint32_t { return op.cost; }).sum());

  return std::pair<uint32_t,std::unique_ptr<ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>>(cur_cost.template bottomRightCorner<1,1>()[0], std::move(output));
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::unique_ptr<ld::ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  ld::reverseAlignment(const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& alignment) noexcept
{
  const uint32_t len = alignment.size();
  const boost::integer_range<uint32_t> inds(0, len);
  std::unique_ptr<ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output(std::make_unique<ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(len));
  std::for_each(std::execution::unseq, inds.begin(), inds.end(),
  [&alignment, &output] (const uint32_t ind) -> void
  {
    switch(alignment[ind].type)
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        (*output)[ind] = EditOp<EIG_OPTIONS>(alignment[ind].cost,
                                             alignment[ind].corresponding_ind,
                                             alignment[ind].effected_ind,
                                             alignment[ind].new_action,
                                             alignment[ind].old_action);
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
        (*output)[ind] = EditOp<EIG_OPTIONS>(alignment[ind].cost,
                                             alignment[ind].new_action,
                                             alignment[ind].corresponding_ind,
                                             (0 == alignment[ind].effected_ind) ? 0 : alignment[ind].effected_ind - 1);
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        (*output)[ind] = EditOp<EIG_OPTIONS>(alignment[ind].cost,
                                             alignment[ind].corresponding_ind + 1,
                                             alignment[ind].effected_ind,
                                             alignment[ind].old_action);
        break;
      default:
        assert(false);
        break;
    }
  });
  assert(std::is_sorted(output->cbegin(), output->cend(),
                        [] (const ld::EditOp<EIG_OPTIONS>& lhs, const ld::EditOp<EIG_OPTIONS>& rhs) -> bool
                        {
                          return  (lhs.effected_ind >  rhs.effected_ind) or
                                 ((lhs.effected_ind == rhs.effected_ind) and (lhs.corresponding_ind > rhs.corresponding_ind));
                        }));
  return output;
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  ld::applyEditOp(const EditOp<EIG_OPTIONS>&                    edit_op,
                  const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& plan) noexcept
{
  const uint32_t plan_len = plan.size();
  const uint32_t size_diff = 0
                             + ((edit_op.type == EditOp<EIG_OPTIONS>::Type::Insertion)     ? 1                                                           : 0)
                             - ((edit_op.type == EditOp<EIG_OPTIONS>::Type::Deletion)      ? 1                                                           : 0);

  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output     = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(plan_len + size_diff);
  Eigen::Index                                            output_ind = 0;
//  bool                                   did_op = false;
  for(uint32_t plan_ind = 0; plan_ind < plan_len; ++plan_ind)
  {
    if(edit_op.effected_ind == plan_ind)
    {
//      did_op = true;
      switch(edit_op.type)
      {
        case EditOp<EIG_OPTIONS>::Type::Substitution:
          (*output)[output_ind] = edit_op.new_actions[0];
          ++output_ind;
          break;
        case EditOp<EIG_OPTIONS>::Type::Insertion:
          (*output)[output_ind] = edit_op.new_actions[0];
          ++output_ind;
          // Do normal copy
          (*output)[output_ind] = plan[plan_ind];
          ++output_ind;
          break;
        case EditOp<EIG_OPTIONS>::Type::Deletion:
          break;
        default:
          assert(false);
          break;
      }
    }
    else // Copy old plan as is
    {
      (*output)[output_ind] = plan[plan_ind];
      ++output_ind;
    }
  }
  if(edit_op.effected_ind == plan_len)
  {
//    did_op = true;
    assert(EditOp<EIG_OPTIONS>::Type::Insertion == edit_op.type);
    (*output)[output_ind] = edit_op.new_actions[0];
    ++output_ind;
  }
//  assert(did_op);
  return output;
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  ld::applyAlignment(const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& alignment,
                     const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&      plan) noexcept
{
  const Eigen::Index num_edit_ops = alignment.size();
  #ifndef NDEBUG
    for(Eigen::Index op_ind = 1; op_ind < num_edit_ops; ++op_ind)
    {
      assert(( alignment[op_ind - 1].effected_ind      <  alignment[op_ind].effected_ind) or
             ((alignment[op_ind - 1].effected_ind      == alignment[op_ind].effected_ind) and
              (alignment[op_ind - 1].corresponding_ind <  alignment[op_ind].corresponding_ind)));
    }
  #endif

  const uint32_t plan_len = plan.size();

  const uint32_t size_diff = std::accumulate<typename ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator,uint32_t>(
                             alignment.cbegin(),
                             alignment.cend(),
                             0,
                             [] (const uint32_t prev_val, const EditOp<EIG_OPTIONS>& edit_op) -> uint32_t
                             {
                               return prev_val + ((edit_op.type == EditOp<EIG_OPTIONS>::Type::Insertion) ? 1 : 0)
                                               - ((edit_op.type == EditOp<EIG_OPTIONS>::Type::Deletion)  ? 1 : 0);
                             });

  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output   = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(plan_len + size_diff);
  Eigen::Index                                            out_ind  = 0;
  uint32_t                                                plan_ind = 0;
  for(Eigen::Index op_ind = 0; op_ind < num_edit_ops; ++op_ind)
  {
    const EditOp<EIG_OPTIONS>& edit_op = alignment[op_ind];

    // Get to edit operation position
    for( ; plan_ind < edit_op.effected_ind; ++plan_ind)
    {
      // Do normal copy
      (*output)[out_ind++] = plan[plan_ind];
    }
    // Perform operation
    switch(edit_op.type)
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        (*output)[out_ind++] = edit_op.new_actions[0];
        ++plan_ind;
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
//        // Do normal copy
//        output.emplace_back(flat_plan[plan_ind].release());
//        ++plan_ind;
        (*output)[out_ind++] = edit_op.new_actions[0];
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        ++plan_ind;
        break;
      default:
        assert(false);
        break;
    }
  }
  // Copy the rest of the plan
  for( ; plan_ind < plan_len; ++plan_ind)
  {
    (*output)[out_ind++] = plan[plan_ind];
  }
  return output;
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,bool>
  ld::performMovement(const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& alignment,
                      const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&      plan,
                      const double                                       max_move_cost,
                      RAND_GEN_TYPE&                                     rand_gen) noexcept
{
  const Eigen::Index num_edit_ops = alignment.size();
  const uint32_t     plan_len     = plan.     size();

  /// Copy current plan
  std::vector<Action> output;
  output.reserve(plan_len + num_edit_ops);
  output.resize(plan_len);
  Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), plan_len) = plan;

  /// Copy operations, second: the effected index, third: number_agent_plan_starts_added
  std::vector<std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>> rand_alignment(num_edit_ops);
  {
    const boost::integer_range<uint32_t> rand_alignment_inds(0, num_edit_ops);
    std::for_each(std::execution::unseq, rand_alignment_inds.begin(), rand_alignment_inds.end(),
    [&] (const uint32_t rand_alignment_ind) -> void
    {
      std::get<0>(rand_alignment[rand_alignment_ind]) = &alignment[rand_alignment_ind];
      std::get<1>(rand_alignment[rand_alignment_ind]) = alignment[rand_alignment_ind].effected_ind;
      std::get<2>(rand_alignment[rand_alignment_ind]) = 0;
      if((EditOp<EIG_OPTIONS>::Type::Insertion == alignment[rand_alignment_ind].type) or (EditOp<EIG_OPTIONS>::Type::Substitution == alignment[rand_alignment_ind].type))
      {
        std::get<2>(rand_alignment[rand_alignment_ind]) += int32_t(nullptr == alignment[rand_alignment_ind].new_action.prev_path);
      }
      if((EditOp<EIG_OPTIONS>::Type::Deletion == alignment[rand_alignment_ind].type) or (EditOp<EIG_OPTIONS>::Type::Substitution == alignment[rand_alignment_ind].type))
      {
        std::get<2>(rand_alignment[rand_alignment_ind]) -= int32_t(nullptr == alignment[rand_alignment_ind].old_action.prev_path);
      }
    });
  }

  /// Find operators to apply and apply them one at a time
  uint32_t summed_cost          = 0;
  int32_t  net_agent_start_diff = 0;
  uint32_t num_ops_performed    = 0;
  std::vector<std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>> ops_need_processing;
  ops_need_processing.reserve(num_edit_ops);
  while(true)
  {
    // Find operation to perform
    std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t> op_to_perf;
    if(not ops_need_processing.empty())
    {
      op_to_perf = ops_need_processing.back();
      ops_need_processing.pop_back();
    }
    else if(net_agent_start_diff > 0)
    {
      // Find a operation that removes an agent start plan action
      const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
      [] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
      {
        return 0 > std::get<2>(ops_ptr);
      });
      assert(rand_alignment.cend() != find_it);
      // Do that operation next
      op_to_perf = std::move(*find_it);
      rand_alignment.erase(find_it);
    }
    else if(net_agent_start_diff < 0)
    {
      // Find a operation that adds an agent start plan action
      const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
      [] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
      {
        return 0 < std::get<2>(ops_ptr);
      });
      assert(rand_alignment.cend() != find_it);
      // Do that operation next
      op_to_perf = std::move(*find_it);
      rand_alignment.erase(find_it);
    }
    else if(nullptr != output.front().prev_path)
    {
      // Find an operation that modifies the first action
      const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
      [] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
      {
        return 0 == std::get<1>(ops_ptr);
      });
      assert(rand_alignment.cend() != find_it);
      // Do that operation next
      op_to_perf = std::move(*find_it);
      rand_alignment.erase(find_it);
    }
    else if(MAX_PLAN_VISITS < output.size())
    {
      assert(Eigen::Dynamic != MAX_PLAN_VISITS);
      // Find a operation that removes an action
      const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
      [] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
      {
        return  (EditOp<EIG_OPTIONS>::Type::Deletion      == std::get<0>(ops_ptr)->type);
      });
      assert(rand_alignment.cend() != find_it);
      // Do that operation next
      op_to_perf = std::move(*find_it);
      rand_alignment.erase(find_it);
    }
    else if((not rand_alignment.empty()) and (double(summed_cost) < max_move_cost))
    {
      std::uniform_int_distribution<uint32_t> op_dist(0, rand_alignment.size() - 1);
      auto to_remove = std::next(rand_alignment.begin(), op_dist(rand_gen));
      op_to_perf = std::move(*to_remove);
      rand_alignment.erase(to_remove);
    }
    else
    {
      break;
    }
    net_agent_start_diff += std::get<2>(op_to_perf);
    summed_cost          += std::get<0>(op_to_perf)->cost;
    ++num_ops_performed;
//    std::cout << "op to perform: " << *op_to_perf->first << std::endl;
//    std::cout << "eff ind: " << op_to_perf->second << std::endl;

    // Update other edits ops effected inds accordingly
    switch(std::get<0>(op_to_perf)->type)
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
        std::for_each(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
        [&op_to_perf] (std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> void
        {
          if( (std::get<1>(ops_ptr) >  std::get<1>(op_to_perf)) or
             ((std::get<1>(ops_ptr) == std::get<1>(op_to_perf)) and (std::get<0>(ops_ptr)->corresponding_ind > std::get<0>(op_to_perf)->corresponding_ind)))
          {
            ++std::get<1>(ops_ptr);
          }
        });
        std::for_each(std::execution::unseq, ops_need_processing.begin(), ops_need_processing.end(),
        [&op_to_perf] (std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> void
        {
          if( (std::get<1>(ops_ptr) >  std::get<1>(op_to_perf)) or
             ((std::get<1>(ops_ptr) == std::get<1>(op_to_perf)) and (std::get<0>(ops_ptr)->corresponding_ind > std::get<0>(op_to_perf)->corresponding_ind)))
          {
            ++std::get<1>(ops_ptr);
          }
        });
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        std::for_each(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
        [&op_to_perf] (std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> void
        {
          if(std::get<1>(ops_ptr) > std::get<1>(op_to_perf))
          {
            --std::get<1>(ops_ptr);
          }
        });
        std::for_each(std::execution::unseq, ops_need_processing.begin(), ops_need_processing.end(),
        [&op_to_perf] (std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> void
        {
          if(std::get<1>(ops_ptr) > std::get<1>(op_to_perf))
          {
            --std::get<1>(ops_ptr);
          }
        });
        break;
      default:
        assert(false);
        break;
    }
    // Check for consistency
    switch(std::get<0>(op_to_perf)->type)
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        if((output.size() != (std::get<1>(op_to_perf) + 1)) and
           (nullptr       != output[std::get<1>(op_to_perf) + 1].prev_path) and
           (std::get<0>(op_to_perf)->new_action.vertex_ind != output[std::get<1>(op_to_perf) + 1].prev_path->from_vertex))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            { return std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) + 1); });
          if(rand_alignment.end() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(find_it);
          }
        }
        if((0       != std::get<1>(op_to_perf)) and
           (nullptr != std::get<0>(op_to_perf)->new_action.prev_path) and
           (std::get<0>(op_to_perf)->new_action.prev_path->from_vertex != output[std::get<1>(op_to_perf) - 1].vertex_ind))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.rbegin(), rand_alignment.rend(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            {
              return (((EditOp<EIG_OPTIONS>::Type::Substitution == std::get<0>(ops_ptr)->type) or (EditOp<EIG_OPTIONS>::Type::Deletion == std::get<0>(ops_ptr)->type)) and (std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) - 1))) or
                     ((EditOp<EIG_OPTIONS>::Type::Insertion == std::get<0>(ops_ptr)->type) and (std::get<1>(ops_ptr) == std::get<1>(op_to_perf)));
            });
          if(rand_alignment.rend() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(std::next(find_it).base());
          }
        }
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
        if((output.size() != std::get<1>(op_to_perf)) and
           (nullptr       != output[std::get<1>(op_to_perf)].prev_path) and
           (std::get<0>(op_to_perf)->new_action.vertex_ind != output[std::get<1>(op_to_perf)].prev_path->from_vertex))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            { return std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) + 1); });
          if(rand_alignment.end() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(find_it);
          }
        }
        if((0       != std::get<1>(op_to_perf)) and
           (nullptr != std::get<0>(op_to_perf)->new_action.prev_path) and
           (std::get<0>(op_to_perf)->new_action.prev_path->from_vertex != output[std::get<1>(op_to_perf) - 1].vertex_ind))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.rbegin(), rand_alignment.rend(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            {
              return (((EditOp<EIG_OPTIONS>::Type::Substitution == std::get<0>(ops_ptr)->type) or (EditOp<EIG_OPTIONS>::Type::Deletion == std::get<0>(ops_ptr)->type)) and (std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) - 1))) or
                     ((EditOp<EIG_OPTIONS>::Type::Insertion == std::get<0>(ops_ptr)->type) and (std::get<1>(ops_ptr) == std::get<1>(op_to_perf)));
            });
          if(rand_alignment.rend() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(std::next(find_it).base());
          }
        }
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        if((output.size() != (std::get<1>(op_to_perf) + 1)) and
           (nullptr       != output[std::get<1>(op_to_perf) + 1].prev_path) and
           (0             != std::get<1>(op_to_perf)) and
           (output[std::get<1>(op_to_perf) - 1].vertex_ind != output[std::get<1>(op_to_perf) + 1].prev_path->from_vertex))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            { return std::get<1>(ops_ptr) == std::get<1>(op_to_perf); });
          if(rand_alignment.end() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(find_it);
          }
          else
          {
            const auto find_it = std::find_if(std::execution::unseq, rand_alignment.rbegin(), rand_alignment.rend(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
              {
                return (((EditOp<EIG_OPTIONS>::Type::Substitution == std::get<0>(ops_ptr)->type) or (EditOp<EIG_OPTIONS>::Type::Deletion == std::get<0>(ops_ptr)->type)) and (std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) - 1))) or
                       ((EditOp<EIG_OPTIONS>::Type::Insertion == std::get<0>(ops_ptr)->type) and (std::get<1>(ops_ptr) == std::get<1>(op_to_perf)));
              });
            if(rand_alignment.rend() != find_it)
            {
              ops_need_processing.emplace_back(*find_it);
              rand_alignment.erase(std::next(find_it).base());
            }
          }
        }
        break;
      default:
        assert(false);
        break;
    }

    // Perform edit operation
    switch(std::get<0>(op_to_perf)->type)
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        assert(output[std::get<1>(op_to_perf)] == std::get<0>(op_to_perf)->old_action);
        output[std::get<1>(op_to_perf)] = std::get<0>(op_to_perf)->new_action;
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
        output.insert(std::next(output.begin(), std::get<1>(op_to_perf)), std::get<0>(op_to_perf)->new_action);
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        assert(output[std::get<1>(op_to_perf)] == std::get<0>(op_to_perf)->old_action);
        output.erase(std::next(output.begin(), std::get<1>(op_to_perf)));
        break;
      default:
        assert(false);
        break;
    }
  }

  #ifdef NDEBUG
  if((0 == num_ops_performed) or (num_edit_ops == num_ops_performed))
  {
    return std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,bool>(std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(), false);
  }
  #endif

  return std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,bool>(std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), output.size(), 1)), true);
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::vector<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>>
  ld::findIntermediateSolutions(const ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& alignment,
                                const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&      plan,
                                RAND_GEN_TYPE&                                     rand_gen) noexcept
{
  const Eigen::Index num_edit_ops = alignment.size();
  const uint32_t     plan_len     = plan.     size();

  std::vector<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>> inter_plans;
  inter_plans.reserve(alignment.size() - 1);

  /// Copy current plan
  std::vector<Action> output;
  output.reserve(plan_len + num_edit_ops);
  output.resize(plan_len);
  Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), plan_len) = plan;

  /// Copy operations, second: the effected index, third: number_agent_plan_starts_added
  std::vector<std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>> rand_alignment(num_edit_ops);
  {
    const boost::integer_range<uint32_t> rand_alignment_inds(0, num_edit_ops);
    std::for_each(std::execution::unseq, rand_alignment_inds.begin(), rand_alignment_inds.end(),
    [&] (const uint32_t rand_alignment_ind) -> void
    {
      std::get<0>(rand_alignment[rand_alignment_ind]) = &alignment[rand_alignment_ind];
      std::get<1>(rand_alignment[rand_alignment_ind]) = alignment[rand_alignment_ind].effected_ind;
      std::get<2>(rand_alignment[rand_alignment_ind]) = 0;
      if((EditOp<EIG_OPTIONS>::Type::Insertion == alignment[rand_alignment_ind].type) or (EditOp<EIG_OPTIONS>::Type::Substitution == alignment[rand_alignment_ind].type))
      {
        std::get<2>(rand_alignment[rand_alignment_ind]) += int32_t(nullptr == alignment[rand_alignment_ind].new_action.prev_path);
      }
      if((EditOp<EIG_OPTIONS>::Type::Deletion == alignment[rand_alignment_ind].type) or (EditOp<EIG_OPTIONS>::Type::Substitution == alignment[rand_alignment_ind].type))
      {
        std::get<2>(rand_alignment[rand_alignment_ind]) -= int32_t(nullptr == alignment[rand_alignment_ind].old_action.prev_path);
      }
    });
  }

  /// Find operators to apply and apply them one at a time
  int32_t net_agent_start_diff = 0;
  std::vector<std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>> ops_need_processing;
  ops_need_processing.reserve(num_edit_ops);
  while(true)
  {
    // Find operation to perform
    std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t> op_to_perf;
    if(not ops_need_processing.empty())
    {
      op_to_perf = ops_need_processing.back();
      ops_need_processing.pop_back();
    }
    else if(net_agent_start_diff > 0)
    {
      // Find a operation that removes an agent start plan action
      const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
      [] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
      {
        return 0 > std::get<2>(ops_ptr);
      });
      assert(rand_alignment.cend() != find_it);
      // Do that operation next
      op_to_perf = std::move(*find_it);
      rand_alignment.erase(find_it);
    }
    else if(net_agent_start_diff < 0)
    {
      // Find a operation that adds an agent start plan action
      const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
      [] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
      {
        return 0 < std::get<2>(ops_ptr);
      });
      assert(rand_alignment.cend() != find_it);
      // Do that operation next
      op_to_perf = std::move(*find_it);
      rand_alignment.erase(find_it);
    }
    else if(nullptr != output.front().prev_path)
    {
      // Find an operation that modifies the first action
      const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
      [] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
      {
        return 0 == std::get<1>(ops_ptr);
      });
      assert(rand_alignment.cend() != find_it);
      // Do that operation next
      op_to_perf = std::move(*find_it);
      rand_alignment.erase(find_it);
    }
    else if(MAX_PLAN_VISITS < output.size())
    {
      assert(Eigen::Dynamic != MAX_PLAN_VISITS);
      // Find a operation that removes an action
      const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
      [] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
      {
        return  (EditOp<EIG_OPTIONS>::Type::Deletion      == std::get<0>(ops_ptr)->type);
      });
      assert(rand_alignment.cend() != find_it);
      // Do that operation next
      op_to_perf = std::move(*find_it);
      rand_alignment.erase(find_it);
    }
    else if(1 < rand_alignment.size())
    {
      // Copy intermediate plan
      inter_plans.emplace_back(std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), output.size(), 1)));
      // Pick next operation
      std::uniform_int_distribution<uint32_t> op_dist(0, rand_alignment.size() - 1);
      auto to_remove = std::next(rand_alignment.begin(), op_dist(rand_gen));
      op_to_perf = std::move(*to_remove);
      rand_alignment.erase(to_remove);
    }
    else
    {
      break;
    }
    net_agent_start_diff += std::get<2>(op_to_perf);

    // Update other edits ops effected inds accordingly
    switch(std::get<0>(op_to_perf)->type)
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
        std::for_each(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
        [&op_to_perf] (std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> void
        {
          if( (std::get<1>(ops_ptr) >  std::get<1>(op_to_perf)) or
             ((std::get<1>(ops_ptr) == std::get<1>(op_to_perf)) and (std::get<0>(ops_ptr)->corresponding_ind > std::get<0>(op_to_perf)->corresponding_ind)))
          {
            ++std::get<1>(ops_ptr);
          }
        });
        std::for_each(std::execution::unseq, ops_need_processing.begin(), ops_need_processing.end(),
        [&op_to_perf] (std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> void
        {
          if( (std::get<1>(ops_ptr) >  std::get<1>(op_to_perf)) or
             ((std::get<1>(ops_ptr) == std::get<1>(op_to_perf)) and (std::get<0>(ops_ptr)->corresponding_ind > std::get<0>(op_to_perf)->corresponding_ind)))
          {
            ++std::get<1>(ops_ptr);
          }
        });
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        std::for_each(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
        [&op_to_perf] (std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> void
        {
          if(std::get<1>(ops_ptr) > std::get<1>(op_to_perf))
          {
            --std::get<1>(ops_ptr);
          }
        });
        std::for_each(std::execution::unseq, ops_need_processing.begin(), ops_need_processing.end(),
        [&op_to_perf] (std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> void
        {
          if(std::get<1>(ops_ptr) > std::get<1>(op_to_perf))
          {
            --std::get<1>(ops_ptr);
          }
        });
        break;
      default:
        assert(false);
        break;
    }
    // Check for consistency
    switch(std::get<0>(op_to_perf)->type)
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        if((output.size() != (std::get<1>(op_to_perf) + 1)) and
           (nullptr       != output[std::get<1>(op_to_perf) + 1].prev_path) and
           (std::get<0>(op_to_perf)->new_action.vertex_ind != output[std::get<1>(op_to_perf) + 1].prev_path->from_vertex))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            { return std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) + 1); });
          if(rand_alignment.end() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(find_it);
          }
        }
        if((0       != std::get<1>(op_to_perf)) and
           (nullptr != std::get<0>(op_to_perf)->new_action.prev_path) and
           (std::get<0>(op_to_perf)->new_action.prev_path->from_vertex != output[std::get<1>(op_to_perf) - 1].vertex_ind))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.rbegin(), rand_alignment.rend(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            {
              return (((EditOp<EIG_OPTIONS>::Type::Substitution == std::get<0>(ops_ptr)->type) or (EditOp<EIG_OPTIONS>::Type::Deletion == std::get<0>(ops_ptr)->type)) and (std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) - 1))) or
                     ((EditOp<EIG_OPTIONS>::Type::Insertion == std::get<0>(ops_ptr)->type) and (std::get<1>(ops_ptr) == std::get<1>(op_to_perf)));
            });
          if(rand_alignment.rend() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(std::next(find_it).base());
          }
        }
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
        if((output.size() != std::get<1>(op_to_perf)) and
           (nullptr       != output[std::get<1>(op_to_perf)].prev_path) and
           (std::get<0>(op_to_perf)->new_action.vertex_ind != output[std::get<1>(op_to_perf)].prev_path->from_vertex))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            { return std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) + 1); });
          if(rand_alignment.end() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(find_it);
          }
        }
        if((0       != std::get<1>(op_to_perf)) and
           (nullptr != std::get<0>(op_to_perf)->new_action.prev_path) and
           (std::get<0>(op_to_perf)->new_action.prev_path->from_vertex != output[std::get<1>(op_to_perf) - 1].vertex_ind))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.rbegin(), rand_alignment.rend(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            {
              return (((EditOp<EIG_OPTIONS>::Type::Substitution == std::get<0>(ops_ptr)->type) or (EditOp<EIG_OPTIONS>::Type::Deletion == std::get<0>(ops_ptr)->type)) and (std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) - 1))) or
                     ((EditOp<EIG_OPTIONS>::Type::Insertion == std::get<0>(ops_ptr)->type) and (std::get<1>(ops_ptr) == std::get<1>(op_to_perf)));
            });
          if(rand_alignment.rend() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(std::next(find_it).base());
          }
        }
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        if((output.size() != (std::get<1>(op_to_perf) + 1)) and
           (nullptr       != output[std::get<1>(op_to_perf) + 1].prev_path) and
           (0             != std::get<1>(op_to_perf)) and
           (output[std::get<1>(op_to_perf) - 1].vertex_ind != output[std::get<1>(op_to_perf) + 1].prev_path->from_vertex))
        {
          const auto find_it = std::find_if(std::execution::unseq, rand_alignment.begin(), rand_alignment.end(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
            { return std::get<1>(ops_ptr) == std::get<1>(op_to_perf); });
          if(rand_alignment.end() != find_it)
          {
            ops_need_processing.emplace_back(*find_it);
            rand_alignment.erase(find_it);
          }
          else
          {
            const auto find_it = std::find_if(std::execution::unseq, rand_alignment.rbegin(), rand_alignment.rend(),
            [&op_to_perf] (const std::tuple<const EditOp<EIG_OPTIONS>*,uint32_t,int32_t>& ops_ptr) -> bool
              {
                return (((EditOp<EIG_OPTIONS>::Type::Substitution == std::get<0>(ops_ptr)->type) or (EditOp<EIG_OPTIONS>::Type::Deletion == std::get<0>(ops_ptr)->type)) and (std::get<1>(ops_ptr) == (std::get<1>(op_to_perf) - 1))) or
                       ((EditOp<EIG_OPTIONS>::Type::Insertion == std::get<0>(ops_ptr)->type) and (std::get<1>(ops_ptr) == std::get<1>(op_to_perf)));
              });
            if(rand_alignment.rend() != find_it)
            {
              ops_need_processing.emplace_back(*find_it);
              rand_alignment.erase(std::next(find_it).base());
            }
          }
        }
        break;
      default:
        assert(false);
        break;
    }

    // Perform edit operation
    switch(std::get<0>(op_to_perf)->type)
    {
      case EditOp<EIG_OPTIONS>::Type::Substitution:
        assert(output[std::get<1>(op_to_perf)] == std::get<0>(op_to_perf)->old_action);
        output[std::get<1>(op_to_perf)] = std::get<0>(op_to_perf)->new_action;
        break;
      case EditOp<EIG_OPTIONS>::Type::Insertion:
        output.insert(std::next(output.begin(), std::get<1>(op_to_perf)), std::get<0>(op_to_perf)->new_action);
        break;
      case EditOp<EIG_OPTIONS>::Type::Deletion:
        assert(output[std::get<1>(op_to_perf)] == std::get<0>(op_to_perf)->old_action);
        output.erase(std::next(output.begin(), std::get<1>(op_to_perf)));
        break;
      default:
        assert(false);
        break;
    }
  }

  return inter_plans;
}
} // plan

#endif
/* levenshtein_distance.hpp */
