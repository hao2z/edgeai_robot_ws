"""Pinned upstream adapter for bounded, persistent low-information frontier memory."""
from pathlib import Path


def apply(root, replace):
    header = 'include/frontier_exploration_ros2/frontier_explorer_core.hpp'
    fields = '''
  struct RepeatMemory {
    std::pair<double, double> point;
    int strikes{0};
    int64_t until{0};
    int64_t touched{0};
    bool pending{false};
    int64_t ready{0};
    double before{0.0};
  };
  std::vector<RepeatMemory> repeat_memory_;
  double repeat_known_area(const std::pair<double, double> & point) const;
  FrontierSequence filter_repeat_frontiers(const FrontierSequence & frontiers);
  void finish_repeat_attempt(const FrontierLike & frontier, double before);
'''
    replace(root, header, 'private:\n  struct DispatchContext', 'private:\n'+fields+'\n  struct DispatchContext')
    replace(root, header, '    std::string action_name;\n  };', '    std::string action_name;\n    double repeat_known_before{0.0};\n  };')
    params = {
        'repeat_guard_enabled': ('bool', 'false', 'as_bool'),
        'repeat_guard_match_distance_m': ('double', '0.35', 'as_double'),
        'repeat_guard_min_gain_m2': ('double', '0.03', 'as_double'),
        'repeat_guard_base_cooldown_s': ('double', '30.0', 'as_double'),
        'repeat_guard_max_cooldown_s': ('double', '240.0', 'as_double'),
    }
    replace(root, header, 'struct FrontierExplorerCoreParams\n{', 'struct FrontierExplorerCoreParams\n{\n'+''.join(f'  {t} {k}{{{v}}};\n' for k,(t,v,_) in params.items()))
    node = 'src/frontier_explorer_node.cpp'
    anchor = '  this->declare_parameter<bool>("frontier_suppression_enabled", true);'
    # Match declaration independently of upstream's default.
    path = root/node
    text = path.read_text()
    line = next(l for l in text.splitlines() if 'declare_parameter<bool>("frontier_suppression_enabled"' in l)
    replace(root, node, line, line+'\n'+''.join(f'  this->declare_parameter<{t}>("{k}", {v});\n' for k,(t,v,_) in params.items()))
    anchor = '  params_.frontier_suppression_enabled = this->get_parameter("frontier_suppression_enabled").as_bool();'
    replace(root, node, anchor, anchor+'\n'+''.join(f'  params_.{k} = this->get_parameter("{k}").{method}();\n' for k,(_,_,method) in params.items()))
    dispatch = 'src/frontier_explorer_core_dispatch.cpp'
    replace(root, dispatch, '    action_name,\n  };', '    action_name,\n    frontier.has_value() ? repeat_known_area(frontier->center_point) : 0.0,\n  };')
    anchor = '  cancel_request_in_progress = false;\n  // Cancel reason must not leak'
    replace(root, dispatch, anchor, '''  if (context.has_value() && context->frontier.has_value() && goal_kind == "frontier") {
    finish_repeat_attempt(*context->frontier, context->repeat_known_before);
  }
'''+anchor)
    replace(root, dispatch, '  frontier_suppression_.reset();', '  frontier_suppression_.reset();\n  repeat_memory_.clear();')
    suppression = 'src/frontier_explorer_core_suppression.cpp'
    replace(root, suppression, '  const int64_t now_ns = callbacks.now_ns();\n  if (!suppression_runtime_active(now_ns)) {\n    return frontiers;', '  const auto candidates = filter_repeat_frontiers(frontiers);\n  const int64_t now_ns = callbacks.now_ns();\n  if (!suppression_runtime_active(now_ns)) {\n    return candidates;')
    replace(root, suppression, '    return frontiers;\n  }\n  suppression->prune_expired', '    return candidates;\n  }\n  suppression->prune_expired')
    replace(root, suppression, 'return suppression->filter_frontiers(frontiers);', 'return suppression->filter_frontiers(candidates);')
    replace(root, suppression, '#include <iomanip>', '#include <algorithm>\n#include <cmath>\n#include <iomanip>')
    code = (Path(__file__).with_name('repeat_guard_core.cpp.inc')).read_text()
    path = root/suppression
    path.write_text(path.read_text()+'\n'+code, encoding='utf-8')

    test = (Path(__file__).with_name('repeat_guard_test.cpp.inc')).read_text()
    anchor = 'TEST(FrontierSuppressionTests, ThresholdCreatesRegionAndClearsAttempt)'
    replace(root, 'test/test_frontier_suppression.cpp', anchor, test+'\n'+anchor)
