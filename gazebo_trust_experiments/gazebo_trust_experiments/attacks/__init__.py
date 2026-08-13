from .modes import DelayedCompositeAttack, FalseClearanceAttack, FalseObstacleAttack, StaleReassertionAttack

ATTACKS = {
    'none': None,
    'false_obstacle': FalseObstacleAttack,
    'false_clearance': FalseClearanceAttack,
    'stale_reassertion': StaleReassertionAttack,
    'delayed_composite': DelayedCompositeAttack,
}
