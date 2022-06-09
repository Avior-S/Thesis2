#!/usr/bin/env python
## Phase 2

import rosbag
import rospy
import pandas as pd

options = None

general_features_options = dict(counter = '# of messages', max_consecutive = 'consecutive messages - maxinum time', mean_consecutive= 'consecutive messages - mean time', var_consecutive = 'consecutive messages - variance time', activeness = '# of active connection', dropped = 'dropped messages', delivered = 'delivered messages', traffic = 'traffic (bytes)', age = 'messages\' age')

# min_consecutive = 'mininum time between consecutive messages', 

specific_features_options = dict(joints = "joint states information", location_x = 'location x', location_y = 'location y', vel_linear_x = 'velocity linear x', vel_angular_z = 'velocity angular z', threads = '# of nodes\' threads', max_cpu_node = 'CPU maximun load by nodes', min_cpu_node = 'CPU minimum load by nodes', mean_cpu_node = 'CPU mean load by nodes', std_cpu_node = 'CPU standard deviation load by nodes', max_virt_mem = 'maximun virtual memory usage by nodes',min_virt_mem = 'minimun virtual memory usage by nodes', mean_virt_mem = 'mean virtual memory usage by nodes', std_virt_mem = 'standard deviation virtual memory usage by nodes', max_real_mem = 'maximun real memory usage by nodes',min_real_mem = 'minimum real memory usage by nodes', mean_real_mem = 'mean real memory usage by nodes', std_real_mem = 'standard deviation real memory usage by nodes', cpu_host = 'CPU host load infomation', mem_host = 'memory host information (aviability & usage)')

def get_general_features_options():
  return general_features_options.values()

def get_specific_features_options():
  return specific_features_options.values()
  
class ExtractFeatures:
  
  #include_nodes = ['/rosout']
  exclude_nodes = ['gazebo','rviz','record', 'rqt', 'rosprofiler', 'rosgrapher']
  include_nodes = ['/amcl', '/twist_mux', '/robot_state_publisher', '/move_base', '/map_odom_broadcaster']
  include_hosts = ['matansar']
# ------------------------------------------------------------ constructor ------------------------------------------------------------  

  def __init__(self, topics, window, specific_features_selection = [], general_features_selection = []):
    self.__features_name = []
    self.__window = window
    self.__topics = sorted(list(topics))
    self.set_sepecific_features_selection(specific_features_selection)
    self.set_general_features_selection(general_features_selection)

# ------------------------------------------------------------ setters ------------------------------------------------------------  

  def set_sepecific_features_selection(self, specific_features_selection):
    #specific_features_selection = sorted(specific_features_selection)
    if (set(specific_features_selection) <= set(get_specific_features_options())):
      self.__specific_selection = sorted(list(specific_features_selection)) # new list 
    else:
      self.__specific_selection = []
      print('ERROR: the specific features selection is not a subset of specific features options')
    
  def set_general_features_selection(self, general_features_selection):
    #general_features_selection = sorted(general_features_selection)
    if (set(general_features_selection) <= set(get_general_features_options())):
      self.__general_selection = sorted(list(general_features_selection)) # new list 
    else:
      self.__general_selection = []
      print('ERROR: the general features selection is not a subset of general features options')
    
  def set_window_size(self, window):
    self.__window = window
    
  def set_topics(self, topics):
    self.__topics = sorted(list(topics))
  
# ------------------------------------------------------------ functionality -----------------------------------------------------------------  


  def generate_features(self, bag_file):
    df = pd.DataFrame()
    bag = rosbag.Bag(bag_file)
    stime = bag.get_start_time() # represented by float
    etime = bag.get_end_time()	 # represented by float
    print("--------- Extract Features on %s..." % (bag_file))
    while (stime + self.__window <= etime):
      #print("stime = %s | etime = %s" % (stime , stime + self.__window))
      features = self.__get_features(bag, stime, stime + self.__window)   
      df = df.append([features])
      stime = stime + self.__window
    print("Finished!")
    bag.close()
    df.columns = self.__features_name
    return df
  
  
# ------------------------------------------------------------ private functions ------------------------------------------------------------  

  # return the features in the same calling's order
  def __get_features(self, bag, stime, etime):
    general_features = self.__get_general_features(bag, stime, etime)
    #print "\t2. extract nav_vel features..."
    nav_vel_features = self.__get_features_nav_vel(bag, stime, etime)
    #ss
    joints_features = self.__get_features_joints_info(bag, stime, etime)
    #print "\t3. extract feedback features..."
    mb_feedback_features = self.__get_features_move_base_feedback(bag, stime, etime)
    #print "\t4. extract node_diagnostic features..."
    # node_diagnostic_features = self.__get_features_node_diagnostic(bag, stime, etime)
    #print "\t5. extract host_diagnostic features..."
    # host_diagnostic_features = self.__get_features_host_diagnostic(bag, stime, etime)
    #print "\t6. extract statistics features..."
    statistics_features = self.__get_features_statistics(bag, stime, etime)
    # return general_features + nav_vel_features + joints_features + mb_feedback_features + node_diagnostic_features + host_diagnostic_features + statistics_features
    return general_features + nav_vel_features + joints_features + mb_feedback_features + statistics_features

  # General type - return the features in the same calling's order
  def __get_general_features(self, bag, stime, etime):
    ret = []
    for topic in self.__topics:
      messages = list(bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics = [topic]))
      if(general_features_options['counter'] in self.__general_selection):
        counter = self.__count_messages(topic, messages)
        ret.append(counter)
      #if(general_features_options['min_consecutive'] in self.__general_selection):
      #  min_consecutive_messages = self.__min_consecutive_time_messages(topic, messages)
      #  ret.append(min_consecutive_messages)
      if(general_features_options['traffic'] in self.__general_selection):
        traffic = self.__get_my_traffic(topic, messages)
        ret.append(traffic)
      if(general_features_options['max_consecutive'] in self.__general_selection):
        max_consecutive_messages = self.__max_consecutive_time_messages(topic, messages)
        ret.append(max_consecutive_messages)
      if(general_features_options['mean_consecutive'] in self.__general_selection):
        mean_consecutive_messages = self.__mean_consecutive_time_messages(topic, messages)
        ret.append(mean_consecutive_messages)
      if(general_features_options['var_consecutive'] in self.__general_selection):
        var_consecutive_messages = self.__var_consecutive_time_messages(topic, messages)
        ret.append(var_consecutive_messages)
      if(general_features_options['age'] in self.__general_selection):
        age_messages = self.__age_messages(topic, messages)
        ret.append(age_messages)
    return ret

  def __get_bytes_size(self, data):
    import StringIO
    buff = StringIO.StringIO()
    data.serialize(buff)
    return len(buff.getvalue())
  
  def __get_my_traffic(self, topic, topic_messages):
    self.__update_features_name("%s My_Traffic(Bytes)" % topic)
    traffics = [self.__get_bytes_size(m) for _, m, _ in topic_messages]
    return sum(traffics)


  # Host_diagnostic - return the features in the same calling's order
  def __get_features_host_diagnostic(self, bag, stime, etime):
    topic = '/host_diagnostic'
    ret = []
    messages = list(bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics = [topic]))
    hosts_messages = self.__get_map_hosts_messages(messages)
    for host_name in self.include_hosts:
      host_messages = hosts_messages[host_name]
      cpu_max_mean_std = self.__get_hosts_max_mean_std_cpu_load(host_name, host_messages)
      mem_max_mean_std = self.__get_hosts_mem_avail_use(host_name, host_messages)
      ret.extend(cpu_max_mean_std)
      ret.extend(mem_max_mean_std)
    return ret

  # Node_diagnostic - return the features in the same calling's order
  def __get_features_node_diagnostic(self, bag, stime, etime):
    topic = '/node_diagnostic'
    ret = []
    messages = list(bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics = [topic]))
    nodes_messages = self.__get_map_nodes_messages(messages)
    for node in self.include_nodes:
      messages = nodes_messages[node]
      threads_counter = self.__get_nodes_count_threads(node, messages)
      max_mean_std_cpu_load = self.__get_nodes_max_mean_std_cpu_load(node, messages)
      max_mean_std_virt_mem = self.__get_nodes_max_mean_std_virt_mem(node, messages)
      max_mean_std_real_mem = self.__get_nodes_max_mean_std_real_mem(node, messages)
      ret.extend(threads_counter)
      ret.extend(max_mean_std_cpu_load)
      ret.extend(max_mean_std_virt_mem)
      ret.extend(max_mean_std_real_mem)
    return ret
    
  # statistics - return the features in the same calling's order  
  def __get_features_statistics(self, bag, stime, etime):
    topic = '/statistics'
    messages = list(bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics = [topic]))
    topic_statistics = []
    for topic in self.__topics:
      topic_messages = filter(lambda (n ,m, t) : m.topic == topic , messages)
      if(general_features_options['activeness'] in self.__general_selection):
        active_connection_counter = self.__count_active_connections(topic, topic_messages)
        topic_statistics.append(active_connection_counter)
      if(general_features_options['dropped'] in self.__general_selection):
        dropped_msgs = self.__get_dropped_msgs(topic, topic_messages)
        topic_statistics.append(dropped_msgs)
      if(general_features_options['delivered'] in self.__general_selection):
        delivered_msgs = self.__get_delivered_msgs(topic, topic_messages)
        topic_statistics.append(delivered_msgs)
      #if(general_features_options['traffic'] in self.__general_selection):
	    #traffic = self.__get_traffic(topic, topic_messages)
	    #topic_statistics.append(traffic)
      #max_mean_std_period = self.__get_max_mean_std_period(topic, topic_messages)
      #max_mean_std_age = self.__get_max_mean_std_age(topic, topic_messages)
    return topic_statistics

  # Nav_vel - return the features in the same calling's order
  def __get_features_joints_info(self, bag, stime, etime):
    topic = '/joint_states'
    ret = []
    messages = list(bag.read_messages(start_time=rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics=[topic]))
    if (specific_features_options['joints'] in self.__specific_selection):
      ret = self.__get_joint_states(messages)
    return ret

    # Nav_vel - return the features in the same calling's order
  def __get_features_nav_vel(self, bag, stime, etime):
    topic = '/nav_vel'
    ret = []
    messages = list(bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics = [topic]))
    if(specific_features_options['vel_linear_x'] in self.__specific_selection):
      vel_linear_x = self.__get_linear_velocity_x(messages)
      ret.append(vel_linear_x)
    if(specific_features_options['vel_angular_z'] in self.__specific_selection):
      vel_angular_z = self.__get_angular_velocity_z(messages)
      ret.append(vel_angular_z)
    return ret
      
  # Movebase_Feedback - return the features in the same calling's order    
  def __get_features_move_base_feedback(self, bag, stime, etime):
    topic = '/move_base/feedback'
    ret = []
    messages = list(bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics = [topic]))
    if(specific_features_options['location_x'] in self.__specific_selection):
      location_x = self.__get_location_x(messages)
      ret.append(location_x)
    if(specific_features_options['location_y'] in self.__specific_selection):
      location_y = self.__get_location_y(messages)
      ret.append(location_y)
    return ret

# ------------------------------------------- auxiliary functions --------------------------------

  def __update_features_name(self, *names):
    for name in names:
      if name not in self.__features_name:
        self.__features_name.append(name)

  def __default_values(self, value, times):
    ret = [value]
    return ret * times

  def __to_seconds(self, t):
    return t.secs + t.nsecs * pow(10,-9)

  def __is_include_nodes(self, node):
    for ex_node in self.exclude_nodes:
      if ex_node in node:
        return False;
    return True;
  
  def __get_consecutive_times(self, messages):
    consecutive_times = []
    for (_, _, t1), (_,_,t2) in zip(messages[:-1], messages[1:]):
      t1 = self.__to_seconds(t1)
      t2 = self.__to_seconds(t2)
      consecutive_times.append(t2-t1)
    return consecutive_times
  
  def __get_map_hosts_messages(self, messages):
    messages = filter(lambda (n,m,t): m.hostname in self.include_hosts, messages) #related just to relevant hosts
    ret = {}
    #initilize hosts in the map
    for host in self.include_hosts:
      ret[host] = []
    #get msgs of each host
    for msg in messages:
      (topic , m, t) = msg
      ret[m.hostname].append(msg)
    return ret
  
  def __get_map_nodes_messages(self, messages):
    messages = filter(lambda (n,m,t): m.node in self.include_nodes, messages) #related just to relevant nodes
    ret = {}
    #initilize nodes in the map
    for node in self.include_nodes:
      ret[node] = []
    #get msgs of each node
    for msg in messages:
      (topic , m, t) = msg
      ret[m.node].append(msg)
    return ret
# -------------------------------------------- features type 0 -------------------------------------------------------------------

  #feature 1
  def __count_messages(self, topic, messages):
    self.__update_features_name("Counter(%s)" % topic)
    return len(messages)

  #feature 2 - not used
  def __min_consecutive_time_messages(self, topic, messages):
    self.__update_features_name("Min_Consecutive(%s)" % topic)
    consecutive_times = self.__get_consecutive_times(messages)
    return min(consecutive_times)

  #feature 3 
  def __max_consecutive_time_messages(self, topic, messages):
    self.__update_features_name("Max_Consecutive(%s)" % topic)
    consecutive_times = self.__get_consecutive_times(messages)
    return max([0] + consecutive_times)

  #feature 4
  def __mean_consecutive_time_messages(self, topic, messages):
    import statistics as stat
    self.__update_features_name("Mean_Consecutive(%s)" % topic)
    consecutive_times = self.__get_consecutive_times(messages)
    if len(consecutive_times) < 1:
      return 0
    return stat.mean(consecutive_times)

  #feature 5
  def __var_consecutive_time_messages(self, topic, messages):
    import statistics as stat
    self.__update_features_name("Var_Consecutive(%s)" % topic)
    consecutive_times = self.__get_consecutive_times(messages)
    if len(consecutive_times) < 2:
      return 0
    return stat.variance(consecutive_times)
  
  #feature 6
  def __age_messages(self, topic, messages):
    import statistics as stat
    NO_AGE_MEAN = 0
    self.__update_features_name("Messages-Age(%s)" % topic)
    ages = []
    for topic, m, t in messages:
      if m._has_header:
        send_time = self.__to_seconds(m.header.stamp) * pow(10,9)
        arrival_time = self.__to_seconds(t) * pow(10,9)
        ages.append(arrival_time - send_time)
      else:
        ages.append(0)
    if len(ages) > 0:
      return stat.mean(ages)
    else:
      return NO_AGE_MEAN
    

# -------------------------------------------- features type 1 -------------------------------------------------------------------


  #feature 10
  def __get_location_x(self, messages):
    self.__update_features_name("location x")
    if len(messages) < 1:
      return 0
    topic, msg, t = messages[-1]
    x = msg.feedback.base_position.pose.position.x
    return x
    
  #feature 11
  def __get_location_y(self, messages):
    self.__update_features_name("location y")
    if len(messages) < 1:
      return 0
    topic, msg, t = messages[-1]
    y = msg.feedback.base_position.pose.position.y
    return y

# -------------------------------------------- features type 2 -------------------------------------------------------------------

  #feature 20
  def __get_linear_velocity_x(self, messages):
    self.__update_features_name("linear velocity x")
    if len(messages) < 1:
      return 0
    topic, msg, t = messages[-1]
    vel_x = msg.linear.x
    return vel_x

  #feature 21
  def __get_angular_velocity_z(self, messages):
    self.__update_features_name("angular velocity z")
    if len(messages) < 1:
      return 0
    topic, msg, t = messages[-1]
    vel_z = msg.angular.z
    return vel_z

  # feature 21
  def __get_joint_states(self, messages):
    def trans(x):
      inf = 10000000
      if x == float('Inf'):
        return inf
      if x == -float('Inf'):
        return -1 * inf
      if x.__str__() == 'nan':
        return inf * inf
      if x.__str__() == '-nan':
        return -1 * inf * inf
      if x == float('NaN') or x == float('naN') or x == float('Nan') or x == float('nan'):
        return inf * inf
      if x == -float('NaN') or x == -float('naN') or x ==  -float('Nan') or x == -float('nan'):
        return -1 * inf * inf
      return x
    names = ['head_pan_joint', 'head_tilt_joint', 'left_finger_joint', 'left_wheel_joint', 'right_finger_joint', 'right_wheel_joint', 'rotation1_joint', 'rotation2_joint', 'shoulder1_joint', 'shoulder2_joint', 'shoulder3_joint','torso_joint', 'wrist_joint']
    info = ["position", "effort", "velocity"]
    for inf in info:
      for name in names:
        self.__update_features_name("%s %s" % (name, inf))
    if len(messages) < 1:
      return [0] * (len(names) * len(info))
    topic, msg, t = messages[-1]
    ret = map(trans, list(msg.position + msg.effort + msg.velocity))
    return ret
    # -------------------------------------------- features type 3 -------------------------------------------------------------------

  # #feature 30
  # def __get_nodes_count_threads(self, node, messages):
  #   import statistics as stat
  #   ret = []
  #   if(specific_features_options['threads'] in self.__specific_selection):
  #     self.__update_features_name("Threads(%s)" % node)
  #     if len(messages) < 1:
  #       ret.append(0)
  #     else:
  #       threads = [m.threads for _, m, _ in messages]
  #       ret.append(stat.mean(threads))
  #   return ret
  #
  # #feature 31
  # def __get_nodes_max_mean_std_cpu_load(self, node, messages):
  #   import statistics as stat
  #   import sys
  #   cpu_loads = reduce(lambda acc,m: acc + list(m[1].cpu_loads), messages, [])
  #   ret = []
  #   if(specific_features_options['max_cpu_node'] in self.__specific_selection):
  #     self.__update_features_name("MaxLoadCPU(%s)" % node)
  #     ret.append(max([0] + cpu_loads))
  #   if(specific_features_options['min_cpu_node'] in self.__specific_selection):
  #     self.__update_features_name("MinLoadCPU(%s)" % node)
  #     ret.append(min([sys.maxint] + cpu_loads))
  #   if(specific_features_options['mean_cpu_node'] in self.__specific_selection):
  #     self.__update_features_name("MeanLoadCPU(%s)" % node)
  #     ret.append(stat.mean(cpu_loads)) if len(cpu_loads) > 0 else ret.append(0)
  #   if(specific_features_options['std_cpu_node'] in self.__specific_selection):
  #     self.__update_features_name("StdLoadCPU(%s)" % node)
  #     ret.append(stat.stdev(cpu_loads)) if len(cpu_loads) > 1 else ret.append(sys.maxint)
  #   return ret
  #
  # #feature 32
  # def __get_nodes_max_mean_std_virt_mem(self, node, messages):
  #   import statistics as stat
  #   import sys
  #   virt_mems = reduce(lambda acc,m: acc + list(m[1].virt_mems), messages, [])
  #   ret = []
  #   if(specific_features_options['max_virt_mem'] in self.__specific_selection):
  #     self.__update_features_name("MaxVirtMem(%s)" % node)
  #     ret.append(max([0] + virt_mems))
  #   if(specific_features_options['min_virt_mem'] in self.__specific_selection):
  #     self.__update_features_name("MinVirtMem(%s)" % node)
  #     ret.append(min([sys.maxint] + virt_mems)) #########################################################################
  #   if(specific_features_options['mean_virt_mem'] in self.__specific_selection):
  #     self.__update_features_name("MeanVirtMem(%s)" % node)
  #     ret.append(stat.mean(virt_mems)) if len(virt_mems) > 0 else ret.append(0)
  #   if(specific_features_options['std_virt_mem'] in self.__specific_selection):
  #     self.__update_features_name("StdVirtMem(%s)" % node)
  #     ret.append(stat.stdev(virt_mems)) if len(virt_mems) > 1 else ret.append(sys.maxint)
  #   return ret
  #
  # #feature 33
  # def __get_nodes_max_mean_std_real_mem(self, node, messages):
  #   import statistics as stat
  #   import sys
  #   real_mems = reduce(lambda acc,m: acc + list(m[1].real_mems), messages, [])
  #   ret = []
  #   if(specific_features_options['max_real_mem'] in self.__specific_selection):
  #     self.__update_features_name("MaxRealMem(%s)" % node)
  #     ret.append(max([0] + real_mems))
  #   if(specific_features_options['max_real_mem'] in self.__specific_selection):
  #     self.__update_features_name("MinRealMem(%s)" % node)
  #     ret.append(min([sys.maxint] + real_mems))
  #   if(specific_features_options['mean_real_mem'] in self.__specific_selection):
  #     self.__update_features_name("MeanRealMem(%s)" % node)
  #     ret.append(stat.mean(real_mems)) if len(real_mems) > 0 else ret.append(0)
  #   if(specific_features_options['std_real_mem'] in self.__specific_selection):
  #     self.__update_features_name("StdRealMem(%s)" % node)
  #     ret.append(stat.stdev(real_mems)) if len(real_mems) > 1 else ret.append(sys.maxint)
  #   return ret


# -------------------------------------------- features type 4 -------------------------------------------------------------------
#
#
#   #feature 40
#   def __get_hosts_max_mean_std_cpu_load(self, host_name, messages):
#     import statistics as stat
#     import sys, multiprocessing
#     features = ["Max", "Min", "Mean", "Std"]
#     ret = []
#     if len(messages) > 0:
#       _, m, _ = messages[0]
#       cpus = len(m.cpu_load)
#     else:
#       cpus = multiprocessing.cpu_count()
#     if(specific_features_options['cpu_host'] in self.__specific_selection):
#       for cpu_id in range(0,cpus):
#     for opt in features:
#       self.__update_features_name("%s(%sCPU%s)" % (host_name, opt,cpu_id))
#       if len(messages) < 1:
#         ret = self.__default_values(0, len(features) * cpus)
# 	# defult for stdev
# 	jumping = 4
# 	for i in range(3,len(features) * cpus, jumping):
# 	  ret[i] = sys.maxint
# 	# defult for min
# 	for i in range(1,len(features) * cpus, jumping):
# 	  ret[i] = sys.maxint
#       else:
# 	for cpu_id in range(0,cpus):
# 	    #need to converte from tuple to list
# 	    cpu_loads = reduce(lambda acc,m: acc + list(m[1].cpu_load[cpu_id].cpu_load), messages, [])
# 	    cpu_max = max(cpu_loads + [0])
# 	    cpu_min = min(cpu_loads +[sys.maxint])
# 	    cpu_mean = stat.mean(cpu_loads) if len(cpu_loads) > 0 else 0
# 	    cpu_std = stat.stdev(cpu_loads) if len(cpu_loads) > 1 else sys.maxint
# 	    ret.extend([cpu_max, cpu_min, cpu_mean, cpu_std])
#     return ret
#
#   #feature 41
#   def __get_hosts_mem_avail_use(self, host_name, messages):
#     import statistics as stat
#     import sys
#     ret = []
#     if(specific_features_options['mem_host'] in self.__specific_selection):
#       self.__update_features_name(
# 	"HostMaxUseMem(%s)" % host_name, "HostMinUseMem(%s)" % host_name, "HostMeanUseMem(%s)" % host_name, "HostStdUseMem(%s)" % host_name,
# 	"HostMaxAvailMem(%s)" % host_name, "HostMinAvailMem(%s)" % host_name, "HostMeanAvailMem(%s)" % host_name, "HostStdAvailMem(%s)" % host_name)
#       if len(messages) < 1:
# 	#usage
# 	ret.append(0)		# max
# 	ret.append(0)		# min
# 	ret.append(0)		# mean
# 	ret.append(0)		# stdev
# 	#available
# 	ret.append(sys.maxint)  # max
# 	ret.append(sys.maxint)	# min
# 	ret.append(sys.maxint)  # mean
# 	ret.append(0)		# stdev (all are max int)
#       else:
# 	phymem_useds = reduce(lambda acc,m: acc + list(m[1].phymem_used), messages, [])
# 	phymem_avails = reduce(lambda acc,m: acc + list(m[1].phymem_avail), messages, [])
# 	#usage
# 	ret.append(max(phymem_useds + [0]))		# max
# 	ret.append(min(phymem_useds + [sys.maxint]))	# min
# 	ret.append(stat.mean(phymem_useds) if len(phymem_useds) > 0 else 0)		# mean
# 	ret.append(stat.stdev(phymem_useds)if len(phymem_useds) > 1 else sys.maxint)	# stdev
# 	#available
# 	ret.append(max(phymem_avails + [0]))		# max
# 	ret.append(min(phymem_avails + [sys.maxint]))	# min
# 	ret.append(stat.mean(phymem_avails) if len(phymem_avails) > 0 else 0)		# mean
# 	ret.append(stat.stdev(phymem_avails)if len(phymem_avails) > 1 else sys.maxint)	# stdev
#     return ret
#
#
# -------------------------------------------- features type 5 -------------------------------------------------------------------


  #feature 50
  def __count_active_connections(self, topic, topic_messages):
    self.__update_features_name('%s Active Connections' % topic)
    connections = []
    counter = 0
    for _, m, _ in topic_messages:
      connection = (m.node_pub, m.node_sub)
      if connection not in connections:
        connections.append(connection)
        counter = counter + 1
    return counter

  #feature 51
  def __get_dropped_msgs(self, topic, topic_messages):
    self.__update_features_name('%s Dropped' % topic)
    dropped_msgs = reduce(lambda acc,(n, m, t): acc + m.dropped_msgs, topic_messages, 0)
    return dropped_msgs

  #feature 52
  def __get_delivered_msgs(self, topic, topic_messages):
    self.__update_features_name('%s Delivered' % topic)
    delivered_msgs = reduce(lambda acc,(n, m, t): acc + m.delivered_msgs, topic_messages, 0)
    return delivered_msgs 

  #feature 53
  def __get_traffic(self, topic, topic_messages):
    self.__update_features_name("%s Traffic(Bytes)" % topic)
    traffics = [m.traffic for _, m, _ in topic_messages]
    return sum(traffics)

  #feature 54
  def __get_max_mean_std_period(self, topic, topic_messages):
    self.__update_features_name("%s Max Period" % topic, "%s Mean Period" % topic, "%s Std Period" % topic)
    if len(topic_messages) == 0:
      return self.__default_values(0,3)
    _, m, _ = topic_messages[-1]
    return [self.__to_seconds(m.period_max), self.__to_seconds(m.period_mean), self.__to_seconds(m.period_stddev)]
    
  #feature 55
  def __get_max_mean_std_age(self, topic, topic_messages):
    self.__update_features_name("%s Max Age" % topic, "%s Mean Age" % topic, "%s Std Age" % topic)
    if len(topic_messages) == 0:
      return self.__default_values(0,3)
    _, m, _ = topic_messages[-1]
    return [self.__to_seconds(m.stamp_age_max), self.__to_seconds(m.stamp_age_mean), self.__to_seconds(m.stamp_age_stddev)]


## --------------------------------------------------------------------------------------------------------------------------------------------
## --------------------------------------------------------------------------------------------------------------------------------------------
## ---------------------------------------------------- main ----------------------------------------------------------------------------------





def write_to_csv(csv_file, df):
  df.to_csv(csv_file, index=False)

def ApplyOptions():
  from optparse import OptionParser
  global options
  parser = OptionParser()
  parser.add_option("-b", "--bagfile", dest="bag_file",help="run BAG file (.bag)", metavar="BAG")
  parser.add_option("-o", "--output", dest="output",help="write report to OUT (.csv)", metavar="OUT")
  #parser.add_option("-p", "--prefix", dest="prefix", help="PR of OUT", default='/home/matansar/catkin_ws/src/robotican/robotican_demos/src/thesis-robotics/RosBagTraining/GoForwordRes/', metavar="PR")
  options, args  = parser.parse_args()
  if not options.bag_file:
    parser.error('BAG file is not given')
  if not options.output:
    parser.error('OUT file is not given')

def print_bag(bag_file, path_file,window):
  bag = rosbag.Bag(bag_file)
  stime = bag.get_start_time() + 0	# represented by float
  etime = stime + window 	# represented by float
  with open(path_file, "w") as f:
    for msg in bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime)):
      f.write("time: " + str(msg[2]) + "\n")
      f.write(str(msg[1]) + "\n\n\n\n")
  



#if __name__ == '__main__':
  #ApplyOptions()
  #dic = dict(threads = '# of nodes\' threads', max_cpu_node = 'CPU maximun load by nodes', min_cpu_node = 'CPU minimum load by nodes', mean_cpu_node = 'CPU mean load by nodes', std_cpu_node = 'CPU standard deviation load by nodes', max_virt_mem = 'maximun virtual memory usage by nodes',min_virt_mem = 'minimun virtual memory usage by nodes', mean_virt_mem = 'mean virtual memory usage by nodes', std_virt_mem = 'standard deviation virtual memory usage by nodes', max_real_mem = 'maximun real memory usage by nodes',min_real_mem = 'minimum real memory usage by nodes', mean_real_mem = 'mean real memory usage by nodes', std_real_mem = 'standard deviation real memory usage by nodes')
  #bag_path = "bags/diagnostic.bag"
  #output_path = "bags/diagnostic.csv"
  #path_file = "bags/diagnostic.info"
  #topics = ['/host_diagnostic']
  #window = 1
  #general_features_selection = []
  ##specific_features_selection  = dic.values()
  #specific_features_selection = ['CPU host load infomation']
  #ef = ExtractFeatures(topics, window, specific_features_selection, general_features_selection)
  #df = ef.generate_features(bag_path)
  #write_to_csv(output_path, df)
  #print_bag(bag_path, path_file,window)

  
  
  
  
  
  
## --------------------------------------------------------------------------------------------------------------------------------------------
## --------------------------------------------------------------------------------------------------------------------------------------------
## ---------------------------------------------------- dont use ------------------------------------------------------------------------------
 

  # def __tmp(self):
  #   def __get_features_host_statistics(self, bag, stime, etime):
  #     topic = '/host_statistics'
  #     messages = list(bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics = [topic]))
  #     cpu_max_mean_std = self.__get_hosts_max_mean_std_cpu_load(messages)
  #     mem_max_mean_std = self.__get_hosts_mem_avail_use(messages)
  #     return cpu_max_mean_std + mem_max_mean_std
  #
  #   # Node_statistics - return the features in the same calling's order
  #   def __get_features_node_statistics(self, bag, stime, etime):
  #     topic = '/node_statistics'
  #     messages = list(bag.read_messages(start_time = rospy.Time.from_sec(stime), end_time=rospy.Time.from_sec(etime), topics = [topic]))
  #     messages = self.__get_relevant_nodes(messages)
  #     threads_counter = self.__get_nodes_count_threads(messages)
  #     max_mean_std_cpu_load = self.__get_nodes_max_mean_std_cpu_load(messages)
  #     max_mean_std_virt_mem = self.__get_nodes_max_mean_std_virt_mem(messages)
  #     max_mean_std_real_mem = self.__get_nodes_max_mean_std_real_mem(messages)
  #     return  threads_counter + max_mean_std_cpu_load + max_mean_std_virt_mem + max_mean_std_real_mem
  #
  #   def __get_relevant_nodes(self, messages):
  #     messages = filter(lambda (n,m,t): m.node in self.include_nodes, messages) #related just to relevant nodes
  #     ret = []
  #     nodes = []
  #     for message in reversed(messages):
	# (topic , m, t) = message
	# node = m.node
	# if node not in nodes:
	#   ret.append(message)
	#   nodes.append(node)
  #     return ret
  #
  #   ##feature 30
  #   def __get_nodes_count_threads(self, messages):
  #     ret = []
  #     if(specific_features_options['threads'] in self.__specific_selection):
	# self.__update_features_name("System Threads")
	# if len(messages) < 1:
	#   ret.append(0)
	# else:
	#   threads = [m.threads for _, m, _ in messages]
	#   ret.append(sum(threads))
  #     return ret
  #
  #   #feature 31
  #   def __get_nodes_max_mean_std_cpu_load(self, messages):
  #     import statistics as stat
  #     ret = []
  #     if(specific_features_options['max_cpu_node'] in self.__specific_selection):
	# self.__update_features_name("Nodes Max CPU")
	# cpu_maxs = [m.cpu_load_max for _, m, _ in messages]
	# ret.append(max([0] + cpu_maxs))
  #     if(specific_features_options['mean_cpu_node'] in self.__specific_selection):
	# self.__update_features_name("Nodes Mean CPU")
	# cpu_means = [m.cpu_load_mean for _, m, _ in messages]
	# ret.append(stat.mean(cpu_means)) if len(cpu_means) > 0 else ret.append(0)
  #     if(specific_features_options['std_cpu_node'] in self.__specific_selection):
	# self.__update_features_name("Nodes Std CPU")
	# cpu_stds = [m.cpu_load_std for _, m, _ in messages]
	# ret.append(stat.mean(cpu_stds)) if len(cpu_stds) > 0 else ret.append(0)
  #     return ret
  #
  #   #feature 32
  #   def __get_nodes_max_mean_std_virt_mem(self, messages):
  #     import statistics as stat
  #     ret = []
  #     if(specific_features_options['max_virt_mem'] in self.__specific_selection):
	# self.__update_features_name("Nodes Max Virt-mem")
	# virl_mem_maxs = [m.virt_mem_max for _, m, _ in messages]
	# ret.append(max([0] + virl_mem_maxs))
  #     if(specific_features_options['mean_virt_mem'] in self.__specific_selection):
	# self.__update_features_name("Nodes Mean Virt-mem")
	# virl_mem_means = [m.virt_mem_mean for _, m, _ in messages]
	# ret.append(stat.mean(virl_mem_means)) if len(virl_mem_means) > 0 else ret.append(0)
  #     if(specific_features_options['std_virt_mem'] in self.__specific_selection):
	# self.__update_features_name("Nodes Std Virt-mem")
	# virl_mem_stds = [m.virt_mem_std for _, m, _ in messages]
	# ret.append(stat.mean(virl_mem_stds)) if len(virl_mem_stds) > 0 else ret.append(0)
  #     return ret
  #
  #   #feature 33
  #   def __get_nodes_max_mean_std_real_mem(self, messages):
  #     import statistics as stat
  #     ret = []
  #     if(specific_features_options['max_real_mem'] in self.__specific_selection):
	# self.__update_features_name("Nodes Max Real-mem")
	# real_mem_maxs = [m.real_mem_max for _, m, _ in messages]
	# ret.append(max([0] + real_mem_maxs))
  #     if(specific_features_options['mean_real_mem'] in self.__specific_selection):
	# self.__update_features_name("Nodes Mean Real-mem")
	# real_mem_means = [m.real_mem_mean for _, m, _ in messages]
	# ret.append(stat.mean(real_mem_means)) if len(real_mem_means) > 0 else ret.append(0)
  #     if(specific_features_options['std_real_mem'] in self.__specific_selection):
	# self.__update_features_name("Nodes Std Real-mem")
	# real_mem_stds = [m.real_mem_std for _, m, _ in messages]
	# ret.append(stat.mean(real_mem_stds)) if len(real_mem_stds) > 0 else ret.append(0)
  #     return ret
  #
  #
	# #feature 40
  #   def __get_hosts_max_mean_std_cpu_load(self, messages):
  #     import statistics as stat
  #     ret = []
  #     if(specific_features_options['cpu_host'] in self.__specific_selection):
	# for opt in ["Max", "Mean", "Std"]:
	#   self.__update_features_name("%s CPU" % (opt,))
	# if len(messages) < 1:
	#   ret = self.__default_values(0,3)
	# else:
	#   _, m, _ = messages[-1]
	#   cpu_max = max(list(m.cpu_load_max))
	#   cpu_mean = stat.mean(list(m.cpu_load_mean))
	#   cpu_std = stat.mean(list(m.cpu_load_std))
	#   ret = [cpu_max, cpu_mean, cpu_std]
  #     return ret
  #
  #   #feature 41
  #
  #   def __get_hosts_mem_avail_use(self, messages):
  #     ret = []
  #     if(specific_features_options['mem_host'] in self.__specific_selection):
	# self.__update_features_name("Hosts Max Use-mem", "Hosts Mean Use-mem", "Hosts Std Use-mem", "Hosts Max Avail-mem", "Hosts Mean Avail-mem", "Hosts Std Avail-mem")
	# if len(messages) < 1:
	#   ret.extend(self.__default_values(0,3)) 	#for usage memory
	#   ret.extend(self.__default_values(-1,2))	#for avail memory (max and mean)
	#   ret.extend(self.__default_values(0,1))	#for avail memory (std)
	# else:
	#   _, m, _ = messages[-1]
	#   ret = [m.phymem_used_max, m.phymem_used_mean, m.phymem_used_std, m.phymem_avail_max, m.phymem_avail_mean, m.phymem_avail_std]
  #     return ret
  #
