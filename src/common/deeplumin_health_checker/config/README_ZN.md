阈值监控设置文件的格式如下：

---

**[1] 必须列出所有要监控的键。**  
若要了解所有可设置的键，请先运行所有必需的节点，然后执行以下命令：
```bash
rosparam dump dump.yaml /diag_reference
```

**[2] 必须根据您的环境设置监控目标阈值。**  
此处未列出的任何键都将被排除在监控之外。

如果您是基于 `dump.yaml` 创建的，请不要忘记在顶部添加命名空间 `health_checker`。

```yaml
health_checker:
  your_monitoring_target1:
    min:
      warn: double_warn_value_threshold
      error: double_error_value_threshold
      fatal: double_fatal_value_threshold
    max:
      warn: double_warn_value_threshold
      error: double_error_value_threshold
      fatal: double_fatal_value_threshold

  your_monitoring_target2:
    min:
      # 若您想使用默认的 warn/error/fatal 值，请移除 warn/error/fatal。
      error: double_error_value_threshold
      fatal: double_fatal_value_threshold

  your_monitoring_target3:
    max: default
      # 若您想在 max/min/rate 中使用所有默认值，请写 "default" 并移除 warn/error/fatal。

  your_monitoring_target4: default
    # 若您想在该键中使用所有默认值，请写 "default" 并移除 max/min/rate。
    ...

  topic_rate_topic1:
    pub_node: /your_node_a
    sub_node: /your_node_b
    topic_name: /your_topic
    rate:
      warn: double_warning_rate_threshold
      error: double_error_rate_threshold
      fatal: double_fatal_rate_threshold

  topic_rate_topic2:
    pub_node: /your_node_a
    sub_node: /your_node_b
    topic_name: /your_topic
    rate: default
    ...
```

**[3] 您可以动态加载这些阈值：**  
```bash
rosparam load this_file
```