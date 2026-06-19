#!/usr/bin/env python3

import rospy
import csv
from rosgraph_msgs.msg import TopicStatistics
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater


STATUS_TO_COLOR = {
    DiagnosticStatus.OK: "white",
    DiagnosticStatus.WARN: "yellow",
    DiagnosticStatus.ERROR: "red",
    DiagnosticStatus.STALE: "lightgray"
}

class TopicMonitor:
    def __init__(self):
        
        # Parameters
        self.monitoring_conf_path = rospy.get_param('~monitoring_conf_path')
        self.ema_gain = rospy.get_param('~ema_gain')  # Exponential moving average gain
        self.hardware_id = rospy.get_param('~hardware_id')
        
        # Publishers
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID(self.hardware_id)
                
        # Subscribers
        rospy.Subscriber('/statistics', TopicStatistics, self.topic_statistics_callback, queue_size=1)
        
        # Other initializations
        # Exponential moving average values for frequency, delay, and other statistics
        self.avg_freqs = {}
        self.avg_delays = {}
        self.delivered_msgs = {}
        
        self.monitoring_config = self.load_monitoring_config()
    
    def load_monitoring_config(self):
        config = {}
        with open(self.monitoring_conf_path, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                topic = row['topic']
                config[topic] = {
                    'component': row['component'],
                    'warning_freq': float(row['warning_freq']),
                    'error_freq': float(row['error_freq']),
                    'warning_delay': float(row['warning_delay']),
                    'error_delay': float(row['error_delay']),
                }                
                self.updater.add(row['component'], lambda stat, m_topic=topic: self.update_diagnostics(stat, m_topic))
        return config

    def update_diagnostics(self, stat, topic):
        if not(topic in self.avg_freqs and topic in self.avg_delays and topic in self.delivered_msgs):
            stat.summary(DiagnosticStatus.STALE, "no data received yet")
            return

        if self.delivered_msgs[topic] == 0:
            stat.summary(DiagnosticStatus.STALE, "no delivered messages")
            return

        avg_freq = self.avg_freqs[topic]            
        freq_status = None
        message = ""
        if avg_freq < self.monitoring_config[topic]['error_freq']:
            message += "frequency error"
            freq_status = DiagnosticStatus.ERROR
        elif avg_freq < self.monitoring_config[topic]['warning_freq']:
            message += "frequency warning"
            freq_status = DiagnosticStatus.WARN
        else:
            message += "frequency OK"
            freq_status = DiagnosticStatus.OK
        
        avg_delay = self.avg_delays[topic]
        delay_status = None
        if avg_delay > self.monitoring_config[topic]['error_delay']:
            message += ", delay error"
            delay_status = DiagnosticStatus.ERROR
        elif avg_delay > self.monitoring_config[topic]['warning_delay']:
            message += ", delay warning"
            delay_status = DiagnosticStatus.WARN
        else:
            message += ", delay OK"
            delay_status = DiagnosticStatus.OK
            
        level = max(freq_status, delay_status)  # Ensure the highest status level is set
        stat.summary(level, message)

        freq_html_string = f"<span style='color: {STATUS_TO_COLOR[freq_status]};'>{avg_freq:.1f} Hz</span>"
        stat.add("Frequency", freq_html_string)

        delay_html_string = f"<span style='color: {STATUS_TO_COLOR[delay_status]};'>{avg_delay:.3f} s</span>"
        stat.add("Delay", delay_html_string)

        return stat

    
    def topic_statistics_callback(self, msg): 
        topic = msg.topic
        if topic in self.monitoring_config:
            
            # Update frequency
            income_freq = 1.0 / msg.period_mean.to_sec() if msg.period_mean.to_sec() > 0 else 0
            self.avg_freqs[topic] = (1 - self.ema_gain) * self.avg_freqs.get(topic, income_freq) + self.ema_gain * income_freq
                        
            # Update delay
            income_delay = msg.stamp_age_mean.to_sec()
            self.avg_delays[topic] = (1 - self.ema_gain) * self.avg_delays.get(topic, income_delay) + self.ema_gain * income_delay
            
            self.delivered_msgs[topic] = msg.delivered_msgs
            
            self.updater.update()
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('topic_monitor')
    monitor = TopicMonitor()
    monitor.run()