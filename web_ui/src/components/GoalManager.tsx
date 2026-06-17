import type { Point3 } from "../utils/trajectory";
import { formatCoordinate } from "../utils/trajectory";

type GoalManagerProps = {
  startPoint: Point3 | null;
  goals: Point3[];
  editingGoalIndex: number | null;
  onEditGoal: (index: number) => void;
  onDeleteGoal: (index: number) => void;
  onCancelEdit: () => void;
};

/**
 * Right-dock panel that lists the start point plus every queued goal with
 * inline edit / delete controls. Editing simply hands the index back to App,
 * which pre-fills the X/Y/Z form and switches the submit button to a "patch"
 * action; deleting reuses the existing pendingPlanRef + queuedGoalsRef chain
 * so the publish-sequence contract stays intact.
 *
 * The outer <section aria-label="地图与目标"> wrapper is preserved so e2e
 * selectors anchored on that aria-label keep working.
 */
export function GoalManager({
  startPoint,
  goals,
  editingGoalIndex,
  onEditGoal,
  onDeleteGoal,
  onCancelEdit,
}: GoalManagerProps) {
  return (
    <section className="dock-section" aria-labelledby="goal-heading" aria-label="地图与目标">
      <div className="section-heading-row">
        <h2 id="goal-heading">起点/目标管理</h2>
        <span>{startPoint ? `1 / ${goals.length}` : `0 / ${goals.length}`}</span>
      </div>
      {!startPoint && goals.length === 0 ? (
        <p className="empty-state">暂无起点和目标点</p>
      ) : (
        <>
          {startPoint ? (
            <dl className="state-grid point-state">
              <dt>起点</dt>
              <dd>
                {formatCoordinate(startPoint.x)}, {formatCoordinate(startPoint.y)},{" "}
                {formatCoordinate(startPoint.z)}
              </dd>
            </dl>
          ) : null}
          {goals.length > 0 ? (
            <ol className="goal-table">
              {goals.map((goal, index) => {
                const isEditing = editingGoalIndex === index;
                return (
                  <li key={`goal-${index}-${goal.x}-${goal.y}-${goal.z}`} className={isEditing ? "is-editing" : ""}>
                    <span>{index + 1}</span>
                    <strong>
                      {formatCoordinate(goal.x)}, {formatCoordinate(goal.y)}, {formatCoordinate(goal.z)}
                    </strong>
                    <div className="goal-actions">
                      <button
                        type="button"
                        className="goal-action-btn"
                        aria-label={isEditing ? `取消编辑第 ${index + 1} 个目标` : `编辑第 ${index + 1} 个目标`}
                        title={isEditing ? "取消编辑" : "载入到表单进行编辑"}
                        onClick={() => (isEditing ? onCancelEdit() : onEditGoal(index))}
                      >
                        {isEditing ? "↶" : "✎"}
                      </button>
                      <button
                        type="button"
                        className="goal-action-btn goal-action-danger"
                        aria-label={`删除第 ${index + 1} 个目标`}
                        title="删除该目标，剩余目标顺序重发到 bridge"
                        onClick={() => onDeleteGoal(index)}
                      >
                        ×
                      </button>
                    </div>
                  </li>
                );
              })}
            </ol>
          ) : null}
        </>
      )}
    </section>
  );
}
