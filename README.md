# Sample codes for the GR-ROSE in ROS2  

## Folders.  

    Examples: sketches for GR-ROSE.  
    Examples-bin: compiled codes of sketches in the examples folder.  

## Info & Tips.  
|About|Tips|  
|:-----------|:------------|  
|Micro-XRCE-DDS-Agent|[Installation Document](https://micro-xrce-dds.readthedocs.io/en/latest/installation.html#installing-the-agent-stand-alone)|  
||[Configuration Document](https://micro-xrce-dds.readthedocs.io/en/latest/agent.html#)|  
||Do not call /opt/ros/VERSION/setup.bash on the terminal for using compiling Micro-XRCE-DDS-Agent, or you will see a core dump message after connected with a dds-client.|  
||Run as a discoverble agnet server: $ MicroXRCEAgent tcp -p 2020 -d|  
|Micro-XRCE-DDS-Gen|[Installation Document](https://micro-xrce-dds.readthedocs.io/en/latest/gen.html#)|  
||Preparing Java environment: sudo apt install openjdk-11-jre-headless |  

Maintained: 27th Nov. 2019  
Created: 18th Nov. 2019  
