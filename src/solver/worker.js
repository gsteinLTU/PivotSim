import { saPlanner }          from './planners/sa.js';
import { rrtPlanner }         from './planners/rrt-connect.js';
import { buildPlannerContext } from './context.js';
import { buildTrajectory }     from './trajectory.js';

const PLANNERS = { sa: saPlanner, rrt: rrtPlanner };
let cancelFlag = false;

self.onmessage = async ({ data }) => {
  if (data.type === 'cancel') { cancelFlag = true; return; }
  if (data.type === 'start') {
    cancelFlag = false;
    const { stairwellParams, boxDims, plannerType = 'sa', plannerConfig } = data;
    try {
      const planner = PLANNERS[plannerType];
      if (!planner) throw new Error(`Unknown planner: ${plannerType}`);
      const context = buildPlannerContext(stairwellParams, boxDims);
      const result  = await planner.plan(
        context, plannerConfig,
        (d) => self.postMessage({ type: 'progress', plannerType, ...d }),
        () => cancelFlag,
      );
      const { segmentTimes, totalTime } = buildTrajectory(result.poses);
      self.postMessage({
        type: cancelFlag ? 'canceled' : 'done',
        poses: result.poses, segmentTimes, totalTime,
        fits: result.fits ?? false,
        tightestIndex: result.tightestIndex ?? 0,
      });
    } catch (err) {
      self.postMessage({ type: 'error', message: err.message });
    }
  }
};
