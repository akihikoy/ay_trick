#!/usr/bin/python3
from core_tool import *
import time
import threading
import numpy as np

def Help():
  return '''Test ay_py update of PR#10.
  Usage:
    > robot
    > test.test_ay_py_pr_10_fix
  '''

def Run(ct, *args):
  arm = 0
  r_name = ct.robot.Name

  q_home = np.array(ct.robot.Q(arm=arm))
  q_target = q_home.copy()
  q_target[0] += 0.30
  q_target2 = q_home.copy()
  q_target2[0] -= 0.05

  # Long trajectory for testing cancellation (10 sec)
  q_traj = [q_home, q_target, q_home]
  t_traj = [0.0, 5.0, 10.0]

  # Short trajectory for next motion (2 sec)
  # (We do not put the first point so that the traj starts at an intermediate stopped position)
  q_traj_next = [q_target2, q_home]
  t_traj_next = [1.0, 2.0]

  CPrint(1, f'=== Test PR#10 Start for Robot: {r_name} (is_sim={ct.robot.is_sim}) ===')
  print(f'q_traj = {q_traj}')
  print(f't_traj = {t_traj}')
  print(f'q_traj_next = {q_traj_next}')
  print(f't_traj_next = {t_traj_next}')

  def check_duration(start_t, min_t, max_t, tag):
    duration = time.time() - start_t
    if min_t <= duration <= max_t:
      CPrint(1, f'  [Time OK] {tag}: {duration:.4f}s (Expected: {min_t}-{max_t}s)')
      return True
    else:
      CPrint(4, f'  [Time FAIL] {tag}: {duration:.4f}s (Expected: {min_t}-{max_t}s)')
      return False

  def try_stop_motion():
    try:
      CPrint(3, f'[{r_name}] Executing StopMotion()')
      t0 = time.time()
      ct.robot.StopMotion(arm=arm)
      CPrint(1, f'[{r_name}] StopMotion accepted ({time.time()-t0:.4f}s).')
    except Exception as e:
      CPrint(4, f'[{r_name}] StopMotion FAILED: {e}')

  # Run a short next motion
  def next_motion(tag):
    try:
      CPrint(3, f'[{r_name}] Executing Next Motion ({tag})')
      t0 = time.time()
      ct.robot.FollowQTraj(q_traj_next, t_traj_next, arm=arm, blocking=True)
      # Duration of next_motion should be approx 2.0s
      check_duration(t0, 1.8, 3.5, f"{tag} execution")

      q_now = np.array(ct.robot.Q(arm=arm))
      diff = np.max(np.abs(q_now - q_home))
      if diff < 0.02:
        CPrint(1, f'[{r_name}] Next Motion finished successfully. [Pos OK] Error: {diff:.5f}')
      else:
        CPrint(4, f'[{r_name}] Next Motion finished BUT Pos Error is LARGE! [Pos FAIL] Error: {diff:.5f}')
        CPrint(4, f'  Expected: {q_home[0]:.4f}, Actual: {q_now[0]:.4f}')
    except Exception as e:
      CPrint(4, f'[{r_name}] Next Motion FAILED ({tag}): {e}')

  def run_traj():
    try:
      CPrint(1, f'[{r_name}] Thread A (Long) started.')
      ct.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=True)
      CPrint(1, f'[{r_name}] Thread A (Long) completed normally (or stopped without error).')
    except Exception as e:
      CPrint(3, f'[{r_name}] Thread A (Long) interrupted/failed as expected: {e}')

  def run_traj_short():
    try:
      CPrint(1, f'[{r_name}] Thread B (Short) started.')
      ct.robot.FollowQTraj(q_traj_next, t_traj_next, arm=arm, blocking=True)
      CPrint(1, f'[{r_name}] Thread B (Short) completed normally.')
    except Exception as e:
      CPrint(3, f'[{r_name}] Thread B (Short) interrupted: {e}')

  # ==========================================
  # Case 0: Normal Execution
  # ==========================================
  CPrint(2, '\n--- Case 0: blocking=True (Full 10s) -> Next ---')
  t_start = time.time()
  ct.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=True)
  check_duration(t_start, 9.5, 12.0, "Case 0 Full Traj")
  next_motion('Case 0')
  time.sleep(0.5)

  # ==========================================
  # Group 1: FollowQTraj(blocking=False) Tests
  # ==========================================

  # Case 1: blocking=False -> StopMotion -> Next
  CPrint(2, '\n--- Case 1: blocking=False -> StopMotion (Immediate) -> Next ---')
  t_start = time.time()
  ct.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=False)
  try_stop_motion()
  # Should be very fast (< 1.0s for stop)
  check_duration(t_start, 0.0, 1.0, "Case 1 Stop")
  next_motion('Case 1')
  time.sleep(0.5)

  # Case 2: blocking=False -> sleep(1.0) -> StopMotion -> Next
  CPrint(2, '\n--- Case 2: blocking=False -> sleep(1.0) -> StopMotion -> Next ---')
  t_start = time.time()
  ct.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=False)
  time.sleep(1.0)
  try_stop_motion()
  # Should be approx 1.0s + overhead
  check_duration(t_start, 1.0, 2.5, "Case 2 Stop")
  next_motion('Case 2')
  time.sleep(0.5)

  # Case 3: blocking=False -> sleep(1.0) -> Next (Override)
  CPrint(2, '\n--- Case 3: blocking=False -> sleep(1.0) -> Next Motion (Override) ---')
  t_start = time.time()
  ct.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=False)
  time.sleep(1.0)
  # Override without explicit Stop
  next_motion('Case 3')
  # Total time for (sleep 1s + next_motion 2s) ~ 3s
  check_duration(t_start, 3.0, 5.0, "Case 3 Total")
  time.sleep(0.5)

  # Case 4: blocking=False -> Next (Immediate Override)
  CPrint(2, '\n--- Case 4: blocking=False -> Next Motion (Immediate Override) ---')
  t_start = time.time()
  ct.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=False)
  next_motion('Case 4')
  # Total time for (next_motion 2s) ~ 2s
  check_duration(t_start, 2.0, 3.5, "Case 4 Total")
  time.sleep(0.5)

  # ==========================================
  # Group 2: Threaded Execution (blocking=True in thread) Tests
  # This verifies if the lock is released properly to allow cancellation.
  # ==========================================

  # Case 5: Thread -> StopMotion (Immediate)
  CPrint(2, '\n--- Case 5: Thread(blocking=True) -> StopMotion (Immediate) -> Next ---')
  t_start = time.time()
  t = threading.Thread(target=run_traj)
  t.start()
  try_stop_motion() # This requires the lock. If deadlocked, this blocks until traj ends (10s).
  t.join()
  # Should be fast, definitely not 10s.
  check_duration(t_start, 0.0, 2.5, "Case 5 Thread Join")
  next_motion('Case 5')
  time.sleep(0.5)

  # Case 6: Thread -> sleep(1.) -> StopMotion
  CPrint(2, '\n--- Case 6: Thread(blocking=True) -> sleep(1.0) -> StopMotion -> Next ---')
  t_start = time.time()
  t = threading.Thread(target=run_traj)
  t.start()
  time.sleep(1.0)
  try_stop_motion()
  t.join()
  # Should be approx 1s.
  check_duration(t_start, 1.0, 2.5, "Case 6 Thread Join")
  next_motion('Case 6')
  time.sleep(0.5)

  # Case 7: Thread -> sleep(1.) -> Next (Override)
  CPrint(2, '\n--- Case 7: Thread(blocking=True) -> sleep(1.0) -> Next (Override) ---')
  t_start = time.time()
  t = threading.Thread(target=run_traj)
  t.start()
  time.sleep(1.0)
  # next_motion calls FollowQTraj(stop_before_start=True), which calls StopMotion internally.
  next_motion('Case 7')
  t.join()
  # Total time dominated by (sleep 1s + next_motion 2s) ~ 3s.
  # If deadlock occurred, t.join() would wait for the first 10s traj to finish.
  check_duration(t_start, 3.0, 5.0, "Case 7 Total")
  time.sleep(0.5)

  # Case 8: Thread -> Next (Immediate Override)
  CPrint(2, '\n--- Case 8: Thread(blocking=True) -> Next (Immediate Override) ---')
  t_start = time.time()
  t = threading.Thread(target=run_traj)
  t.start()
  next_motion('Case 8')
  t.join()
  # Total time dominated by next_motion 2s.
  check_duration(t_start, 2.0, 3.5, "Case 8 Total")

  CPrint(1, '--- Cleanup after Case 8 ---')
  try_stop_motion()
  time.sleep(0.5)

  # ==========================================
  # Group 3: Advanced & Stress Tests
  # ==========================================

  # Case 9: Test stop_before_start=False (Motion Blending/Override)
  # Verifies that setting stop_before_start=False correctly updates
  # the motion counter and preempts the previous motion via ActionServer
  # without explicit _StopMotion call.
  CPrint(2, '\n--- Case 9: Thread(blocking=True) -> Next(stop_before_start=False) ---')
  t_start = time.time()
  t = threading.Thread(target=run_traj)
  t.start()
  # Wait slightly to ensure thread starts
  time.sleep(0.2)

  try:
    CPrint(3, f'[{r_name}] Executing Next Motion (Case 9, stop_before_start=False)')
    # Override without explicit stop.
    # TEST If the old thread fails due to the ActionServer will preempt it,
    # and the motion counter mismatch will catch any retry attempts.
    ct.robot.FollowQTraj(q_traj_next, t_traj_next, arm=arm, blocking=True, stop_before_start=False)

    # Duration should be approx 2.0s (Next motion duration)
    # If the old motion (10s) wasn't preempted, this would take longer.
    check_duration(t_start, 2.0, 3.5, "Case 9 Total")

  except Exception as e:
    CPrint(4, f'[{r_name}] Next Motion FAILED (Case 9): {e}')

  t.join()
  time.sleep(0.5)

  # Case 10: Thread vs Thread Race (Background A vs Background B)
  # Verifies motion counter consistency when two background threads compete.
  # The last one started should win.
  CPrint(2, '\n--- Case 10: Thread A vs Thread B Race ---')

  t_start = time.time()
  t_a = threading.Thread(target=run_traj)      # Long motion (10s)
  t_b = threading.Thread(target=run_traj_short) # Short motion (2s)

  t_a.start()
  time.sleep(0.1) # Ensure A grabs lock first
  t_b.start()     # B should override A

  t_a.join()
  t_b.join()

  # Total time should be dictated by Thread B (approx 2s + 0.1s delay)
  # If A persists, it would be 10s.
  check_duration(t_start, 1.8, 3.5, "Case 10 Total")
  time.sleep(0.5)

  # Case 11: Stress Test (Repeated Immediate Overrides)
  # Detects potential deadlocks or resource exhaustion by rapid locking/unlocking.
  CPrint(2, '\n--- Case 11: Stress Test (5x Immediate Override) ---')

  success_count = 0
  for i in range(5):
    CPrint(3, f'-- Iteration {i+1}/5 --')
    t = threading.Thread(target=run_traj)
    t.start()

    try:
      # Immediate override
      ct.robot.FollowQTraj(q_traj_next, t_traj_next, arm=arm, blocking=True)
      success_count += 1
    except Exception as e:
      CPrint(4, f'Iteration {i+1} FAILED: {e}')

    t.join()

    # Simple check to ensure we are at home
    q_now = np.array(ct.robot.Q(arm=arm))
    if np.max(np.abs(q_now - q_home)) > 0.05:
      CPrint(4, f'Iteration {i+1} Pos Error! Robot might be lost.')

  if success_count == 5:
    CPrint(1, f'  [Stress OK] 5/5 iterations passed successfully.')
  else:
    CPrint(4, f'  [Stress FAIL] Only {success_count}/5 iterations passed.')

  CPrint(1, f'=== All Tests Finished for {r_name} ===')
