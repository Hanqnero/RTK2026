from rtk2026_graph.global_planner_v2 import GlobalPlannerConfigV2, GlobalPlannerV2


def _planner() -> GlobalPlannerV2:
    cfg = GlobalPlannerConfigV2(
        lane_targets={
            "lane1": {
                17: (20, 19, 18, 2, 4),
                18: (20, 19, 17, 8, 6),
                19: (20, 18, 17, 12, 10),
                20: (19, 18, 17, 14, 16),
                8: (18, 6),
            },
            "lane2": {
                17: (20, 19, 18, 4, 2),
                19: (20, 18, 17, 12, 10),
                20: (19, 18, 17, 14, 16),
                18: (8, 6),
                8: (10,),
            },
        },
        intersection_vertex_ids=frozenset({17, 18, 19, 20}),
        lane_switch_edges=frozenset({(18, 8)}),
        intersection_exit_lane_targets={
            "lane1": {
                17: (2,),
                18: (6,),
                19: (10,),
                20: (14,),
            },
            "lane2": {
                17: (4,),
                18: (8,),
                19: (12,),
                20: (16,),
            },
        },
        initial_lane_mode="lane1",
    )
    return GlobalPlannerV2(cfg)


def test_defaults_to_lane1() -> None:
    p = _planner()
    assert p.initial_lane_mode == "lane1"


def test_sign_target_wins_when_allowed() -> None:
    p = _planner()
    step = p.pick_next(
        current_vertex=18,
        previous_vertex=19,
        active_lane_mode="lane1",
        sign_target_vertex=8,
        visit_counts={8: 10, 6: 0},
    )
    assert step.chosen_target == 8
    assert step.pick_source == "sign"


def test_exit_target_sets_next_lane_mode_by_configured_side() -> None:
    p = _planner()
    step1 = p.pick_next(
        current_vertex=18,
        previous_vertex=19,
        active_lane_mode="lane1",
        sign_target_vertex=8,
    )
    assert step1.chosen_target == 8
    assert step1.lane_switched is True
    assert step1.next_lane_mode == "lane2"

    step2 = p.pick_next(
        current_vertex=8,
        previous_vertex=18,
        active_lane_mode=step1.next_lane_mode,
        sign_target_vertex=-1,
        visit_counts={10: 0, 6: 0},
    )
    assert step2.allowed_targets == (10,)
    assert step2.chosen_target == 10


def test_blocks_immediate_backtrack_when_has_alternative() -> None:
    p = _planner()
    allowed = p.allowed_targets(current_vertex=8, previous_vertex=18, active_lane_mode="lane1")
    assert allowed == (6,)


def test_intersection_exit_filters_to_outer_targets() -> None:
    p = _planner()
    # На ромбе: previous тоже на ромбе => только выезд наружу.
    allowed = p.allowed_targets(current_vertex=18, previous_vertex=19, active_lane_mode="lane1")
    assert allowed == (8, 6)
    assert p.allowed_targets(current_vertex=17, previous_vertex=20, active_lane_mode="lane1") == (2, 4)
    assert p.allowed_targets(current_vertex=19, previous_vertex=20, active_lane_mode="lane1") == (12, 10)
    assert p.allowed_targets(current_vertex=20, previous_vertex=19, active_lane_mode="lane1") == (14, 16)


def test_exit_to_right_target_forces_lane1() -> None:
    p = _planner()
    step = p.pick_next(
        current_vertex=17,
        previous_vertex=20,
        active_lane_mode="lane2",
        sign_target_vertex=2,
    )
    assert step.chosen_target == 2
    assert step.next_lane_mode == "lane1"
    assert step.lane_switched is True
