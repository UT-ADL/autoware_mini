#! /bin/bash -e

# .............
# Description: 
# .............

# Runs CARLA's leaderboard benchmarks and mails results to given recipients. 
# This script is mainly called by the cron job that runs on a remote server. It runs both
# CARLA and ROS stack in background (no GUI). If no recipient is provided, then mail is not
# sent, only results are displayed.

# USAGE: ./nightly_leaderboard.sh <GPU_number> [recipient1@example.com] [recipient2@example.com ...]

# =========================< Helper Methods >==================================

# Check if argument is not a number
is_nan() {
  if ! [[ $1 =~ ^[0-9]+$ ]]; then
    return 0  # It's not a number
  else
    return 1  # It's a number
  fi
}

# Check if any email address is not valid
check_email_validity() {
  local valid=true

  for email in "$@"; do
    if ! [[ $email =~ ^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,4}$ ]]; then
      echo -e "\nInvalid email address: $email"
      valid=false
    fi
  done

  if [ "$valid" = "false" ]; then
    return 1  # Return an error status
  else
    return 0  # All email addresses are valid
  fi
}

# ==============================================================================

CARLA_GPU=${1:-0}

if [ $(is_nan "$CARLA_GPU")] || [ "$CARLA_GPU" -lt 0 ] || [ "$CARLA_GPU" -ge "$(nvidia-smi --list-gpus | wc -l)" ]; then
    echo "Incorrect GPU selected. Script terminated!"
    exit 1
fi

echo -e "=================================="
echo -e "Starting Leaderboard Evaluations.."
echo -e "=================================="

# Run CARLA in background on port 2000 with selected GPU
$CARLA_ROOT/CarlaUE4.sh -RenderOffScreen -carla-server -carla-rpc-port=2000 carla-primary-port=2000 -carla-primary-host=127.0.0.1 -ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=$CARLA_GPU &

# Wait for CARLA to start
sleep 8

# Remove old results if they exist
rm -f ${LEADERBOARD_ROOT}/results.json

# Run CARLA Leaderboard
$TEAM_CODE_ROOT/autoware_mini/scripts/leaderboard/run_evaluations.sh

# Save leaderboard results in a pretty format
python ${LEADERBOARD_ROOT}/scripts/pretty_print_json.py -f ${LEADERBOARD_ROOT}/results.json -o ${LEADERBOARD_ROOT}/leaderboard_eval_$(date +%d-%m-%Y).txt


# =============< Mail Results >================

# Shift to remove the first argument (GPU) from the argument list
shift

# Check validitiy of email recipient(s) and send mail.
if [ $# -gt 0 ] && check_email_validity "$@"; then
  {
  echo "<pre>"
  echo -e "\n===================\nEvaluation Results\n===================\n"
  cat ${LEADERBOARD_ROOT}/leaderboard_eval_$(date +%d-%m-%Y).txt
  echo "</pre>"
    } | mutt -e "set content_type=text/html" -s "[Leaderboard Evaluations]" -e 'my_hdr From: Leaderboard <leaderboard@neuron.hpc.ut.ee>' -- "$@"
else
  echo -e "\nMail Usage: ./nightly_leaderboard.sh <GPU_number> [recipient1@example.com] [recipient2@example.com ...] \n"
fi

# ==============================================

# Display results
echo -e "\n===================\nEvaluation Results\n===================\n"
cat ${LEADERBOARD_ROOT}/leaderboard_eval_$(date +%d-%m-%Y).txt

# Kill background process
killall -9 'CarlaUE4-Linux-' & killall -9 'rosmaster'
