// vuejs object
var app = new Vue({
  el: "#app",

  // member variables
  data: {
    // robot details
    robotName: "CafeteriaBot",

    // ros connection details
    isConnected: false,
    ros: null,
    isLoading: false,
    rosbridgeAddress: "",
    port: "9090",

    // log management
    logArray: [{ level: 0, msg: "Ready to connect!" }],
    maxLogEntries: 100,
    trimSize: 20,

    // parameters
    isParameterServiceBusy: false,
    parameters: {
      maxDistance: { name: "detection_node:max_distance", value: 0.0 },
      minDistance: { name: "detection_node:min_distance", value: 0.0 },
      angleTolerance: { name: "detection_node:angle_tolerance", value: 0 },
      maxPoints: { name: "clustering_node:max_points", value: 0 },
      minPoints: { name: "clustering_node:min_points", value: 0 },
      markerDuration: { name: "detection_node:marker_duration", value: 0 },
    },

    // position & movement
    robotPose: { x: 0.0, y: 0.0, yaw: 0.0 },
    position: { x: 0.0, y: 0.0, z: 0.0 },
    velocity: { x: 0.0, y: 0.0, z: 0.0 },

    // table geometry
    tableGeometry: {
      center: {},
      footprint: [],
      midpoints: [],
    },

    // canvas scale
    canvasScale: 300,
    showTable: false,

    // map
    mapData: {
      header: null,
      info: null,
      data: null,
    },
    showMap: false,

    // services
    controlService: {
      isServiceBusy: false,
      serviceRequest: "start",
    },
    elevatorService: {
      isServiceBusy: false,
      serviceRequest: true,
    },
    tableService: {
      isServiceBusy: false,
      serviceRequest: 5.0,
    },

    // actions
    dockAction: {
      status: { active: false, button: false },
      feedback: { stage: 0, status: 0, success: null, message: "" },
    },

    // progress
    progress: null,
    progress_count: 0,

    // commands
    commands: [
      { name: "Command", command: "none" }, // placeholder
      { name: "Patrol", command: "patrol" },
      { name: "Go Home", command: "drop_to_home" },
      { name: "Go Drop", command: "home_to_drop" },
      { name: "Stop", command: "stop" },
    ],

    // dragging for ui interactions
    isDragging: { x: "no", y: "no", status: false },
    draggingStyle: {
      margin: "0px",
      top: "0px",
      left: "0px",
      display: "inline-block",
      width: "75px",
      height: "75px",
    },

    // joystick controls
    joystick: { x: 0.0, z: 0.0 },
  },

  watch: {
    "canvasScale": function (newVal, oldVal) {
      if (newVal !== oldVal) {
        this.drawTableCanvas(); // redraw when canvasScale changes
      }
    },
    "parameters.minDistance.value": function (newVal, oldVal) {
      if (newVal !== oldVal) {
        this.drawTableCanvas(); // redraw when minDistance changes
      }
    },
    "parameters.maxDistance.value": function (newVal, oldVal) {
      if (newVal !== oldVal) {
        this.drawTableCanvas(); // redraw when maxDistance changes
      }
    },
    "tableGeometry.center": function (newVal, oldVal) {
      if (newVal !== oldVal) {
        this.drawTableCanvas(); // redraw when tableGeometry changes
      }
    },
    "dockAction.feedback.status": function (newVal, oldVal) {
      if (newVal !== oldVal) {
        if (newVal !== 1) {
          this.dockAction.status.button = false; // update button based on status
        }
      }
    },
  },

  computed: {
    progressBarProgress() {
      // only compute progress if there is an active goal
      if (this.dockAction.status.active) {
        if (this.dockAction.feedback.status == 4) {
          this.progress = 100;
          return this.progress;
        } else {
          this.progress_count = this.dockAction.feedback.stage + 1;
          this.progress = this.progress_count * 15;
          return this.progress;
        }
      }
      this.progress = 0;
      return this.progress; // default to 0% progress
    },
    progressBarStyle() {
      return {
        width: `${this.progressBarProgress}%`,
      };
    },
    progressBarClasses() {
      let statusClass;
      switch (this.dockAction.feedback.status) {
        case 0:
          statusClass = "bg-info";
          break;
        case 1:
          statusClass = "bg-info";
          break;
        case 2:
          statusClass = "bg-warning";
          break;
        case 3:
          statusClass = "bg-danger";
          break;
        case 4:
          statusClass = "bg-success";
          break;
        default:
          statusClass = "bg-light";
      }

      return {
        "progress-bar-striped": true,
        "progress-bar-animated": this.dockAction.feedback.success == false,
        [statusClass]: true,
      };
    },
    progressBarText() {
      return this.dockAction.status.active ? `${this.progressBarProgress}%` : "Currently there is no active goal.";
    },
  },

  // member method
  methods: {
    // helper methods to connect with ROS
    connect: function () {
      // ros connection and log
      this.isLoading = true;
      this.ros = new ROSLIB.Ros({
        url: this.rosbridgeAddress,
      });

      this.ros.on("connection", () => {
        this.isConnected = true;
        this.isLoading = false;
        this.logMessage(0, `Connected : ${new Date().toTimeString()}`);
      });

      this.ros.on("error", (error) => {
        this.logMessage(0, `Error : ${new Date().toTimeString()}`);
      });

      this.ros.on("close", () => {
        this.isConnected = false;
        this.isLoading = false;
        this.logMessage(0, `Disconnected : ${new Date().toTimeString()}`);
      });

      // subscribe callback to rosout
      let topicRosout = new ROSLIB.Topic({
        ros: this.ros,
        name: "/rosout",
        messageType: "rcl_interfaces/msg/Log",
      });

      topicRosout.subscribe((message) => {
        // list of nodes to include in the logs
        const allowedNodes = ["cafeteria_robot_control_node"];

        if (allowedNodes.includes(message.name)) {
          // only log messages from specified nodes
          this.logMessage(message.level, message.msg);
        }
      });

      // subscribe callback to odometry
      let topicOdometry = new ROSLIB.Topic({
        ros: this.ros,
        name: "/cleaner_2/odom",
        messageType: "nav_msgs/msg/Odometry",
      });

      topicOdometry.subscribe((message) => {
        this.position = message.pose.pose.position;
      });

      // subscribe callback to velocity
      let topicVelocity = new ROSLIB.Topic({
        ros: this.ros,
        name: "/cleaner_2/cmd_vel",
        messageType: "geometry_msgs/msg/Twist",
      });

      topicVelocity.subscribe((message) => {
        this.velocity = {
          x: message.linear.x,
          y: message.linear.y,
          z: message.angular.z,
        };
      });

      // subscribe callback to table
      let topicTable = new ROSLIB.Topic({
        ros: this.ros,
        name: "/table",
        messageType: "cafeteriabot_interface/msg/TableGeometry",
      });

      topicTable.subscribe((message) => {
        this.tableGeometry = {
          center: message.center,
          footprint: message.footprint,
          midpoints: message.midpoints,
        };
      });

      // subscribe callback to velocity
      let topicMap = new ROSLIB.Topic({
        ros: this.ros,
        name: "/map",
        messageType: "nav_msgs/msg/OccupancyGrid",
      });

      topicMap.subscribe((message) => {
        this.mapData = {
          header: message.header,
          info: message.info,
          data: message.data,
        };
        this.logMessage(0, `Received map data: ${message.info.height}x${message.info.width}.`);

        // draw map on canvas
        this.drawMapCanvas();
      });

      // subscribe callback to action feedback
      let topicDockFeedback = new ROSLIB.Topic({
        ros: this.ros,
        name: "/dock_to_table_feedback",
        messageType: "cafeteriabot_interface/msg/DockToTableFeedback",
      });

      topicDockFeedback.subscribe((message) => {
        this.dockAction.feedback = {
          stage: message.stage,
          status: message.status,
          success: message.success,
          message: message.message,
        };

        // update status dock action
        this.dockAction.status = {
          active: true,
          button: true,
        };
      });

      // subscribe callback to pose
      let topicPose = new ROSLIB.Topic({
        ros: this.ros,
        name: "/current_pose",
        messageType: "geometry_msgs/msg/Pose2D",
      });

      topicPose.subscribe((message) => {
        this.robotPose = {
          x: message.x,
          y: message.y,
          yaw: message.theta,
        };

        // update map on canvas
        this.drawMapCanvas();
      });
    },

    // helper methods to disconnect with ROS
    disconnect: function () {
      this.ros.close();

      // if disconnected, clean viewers
      this.cleanUp();
    },

    // handle log message
    logMessage: function (level, msg) {
      if (this.logArray.length >= this.maxLogEntries) {
        // remove the last 20 entries
        this.logArray.splice(-this.trimSize);
      }
      // add the new entry at the top
      let entry = { level: level, msg: msg };
      this.logArray.unshift(entry);
    },

    getLogClass: function (level) {
      switch (level) {
        case 50:
          return "text-danger"; // for critical errors
        case 30:
          return "text-warning"; // for warnings
        case 20:
          return "text-info"; // for informational messages
        case 10:
          return "text-success"; // for success messages
        default:
          return "text-white"; // default, for debug or unknown levels
      }
    },

    // fetch all parameters
    getParameters: function () {
      this.isParameterServiceBusy = true;

      // iterate over all parameters and fetch their values
      const paramsToFetch = Object.keys(this.parameters);
      paramsToFetch.forEach((key) => {
        let rosParam = new ROSLIB.Param({
          ros: this.ros,
          name: this.parameters[key].name,
        });

        // fetch the parameter value
        rosParam.get(
          (value) => {
            this.parameters[key].value = value;
            // check if this is the last parameter to update
            if (key === paramsToFetch[paramsToFetch.length - 1]) {
              this.isParameterServiceBusy = false;
              this.logMessage(10, `All parameter fetch completed.`);
            }
          },
          (err) => {
            this.isParameterServiceBusy = false;
            this.logMessage(10, `Failed to get parameter ${this.parameters[key].name}: ${err}`);
          },
        );
      });
    },

    // set all parameters
    setParameters: function () {
      this.isParameterServiceBusy = true;

      Object.keys(this.parameters).forEach((key) => {
        let rosParam = new ROSLIB.Param({
          ros: this.ros,
          name: this.parameters[key].name,
        });

        rosParam.set(
          this.parameters[key].value,
          // callback for successful parameter setting
          () => {
            this.isParameterServiceBusy = false;
            this.logMessage(10, `Parameter "${key}" set to ${this.parameters[key].value}.`);
          },
          // callback for failed parameter setting
          (err) => {
            this.isParameterServiceBusy = false;
            this.logMessage(50, `Failed to set parameter "${key}": ${err}`);
          },
        );
      });
    },

    // adjust table canvas aspect ratio
    adjustTableCanvasSize: function () {
      const container = this.$refs.idTableCanvasContainer;
      const canvas = this.$refs.idTableCanvas;

      // compute the size to maintain aspect ratio
      const aspectRatio = 1;
      let width = container.clientWidth;
      let height = container.clientHeight;

      if (height > width / aspectRatio) {
        height = width / aspectRatio;
      } else {
        width = height * aspectRatio;
      }

      // set the canvas size
      canvas.width = width;
      canvas.height = height;
    },

    // draw table canvas
    drawTableCanvas: function () {
      const canvas = this.$refs.idTableCanvas;
      const ctx = canvas.getContext("2d");
      const centerX = canvas.width / 2;
      const centerY = canvas.height / 2;

      // clear canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // draw an 'X' marker at the center
      const markerSize = 5;
      ctx.strokeStyle = "black";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(centerX - markerSize, centerY - markerSize);
      ctx.lineTo(centerX + markerSize, centerY + markerSize);
      ctx.moveTo(centerX + markerSize, centerY - markerSize);
      ctx.lineTo(centerX - markerSize, centerY + markerSize);
      ctx.stroke();

      // draw max distance square
      const maxDistanceWidth = this.parameters.maxDistance.value * this.canvasScale;
      const maxDistanceHeight = this.parameters.maxDistance.value * this.canvasScale;
      ctx.strokeStyle = "red";
      ctx.setLineDash([4, 2]);
      ctx.strokeRect(
        centerX - maxDistanceWidth / 2,
        centerY - maxDistanceHeight / 2,
        maxDistanceWidth,
        maxDistanceHeight,
      );

      // draw min distance square
      const minDistanceWidth = this.parameters.minDistance.value * this.canvasScale;
      const minDistanceHeight = this.parameters.minDistance.value * this.canvasScale;
      ctx.strokeStyle = "green";
      ctx.setLineDash([4, 2]);
      ctx.strokeRect(
        centerX - minDistanceWidth / 2,
        centerY - minDistanceHeight / 2,
        minDistanceWidth,
        minDistanceHeight,
      );

      if (this.showTable) {
        // draw a circle for each footprint point
        this.tableGeometry.footprint.forEach((point) => {
          const x = centerX + (point.x - this.tableGeometry.center.x) * this.canvasScale;
          const y = centerY + (point.y - this.tableGeometry.center.y) * this.canvasScale;

          ctx.fillStyle = "blue";
          ctx.beginPath();
          ctx.arc(x, y, 5, 0, 2 * Math.PI);
          ctx.fill();
        });

        // draw a circle for each midpoints point
        this.tableGeometry.midpoints.forEach((point) => {
          const x = centerX + (point.x - this.tableGeometry.center.x) * this.canvasScale;
          const y = centerY + (point.y - this.tableGeometry.center.y) * this.canvasScale;

          ctx.fillStyle = "orange";
          ctx.beginPath();
          ctx.arc(x, y, 5, 0, 2 * Math.PI);
          ctx.fill();
        });
      }

      // color and description for points
      const legendItems = [
        { color: "black", text: "Center" },
        { color: "red", text: "Outerbound" },
        { color: "green", text: "Innerbound" },
        { color: "blue", text: "Footprint" },
        { color: "orange", text: "Midpoint" },
      ];

      let legendX = 5;
      legendItems.forEach((item) => {
        ctx.fillStyle = item.color;
        ctx.fillRect(legendX, 20, 10, 10);
        ctx.fillStyle = "black";
        ctx.fillText(item.text, legendX + 15, 29);
        legendX += ctx.measureText(item.text).width + 32;
      });
    },

    // clear table canvas
    clearTableCanvas: function () {
      const canvas = this.$refs.idTableCanvas;
      const ctx = canvas.getContext("2d");

      // clear canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // remove table and update
      this.showTable = false;
      this.drawTableCanvas();
    },

    // adjust map canvas aspect ratio
    adjustMapCanvasSize: function () {
      const container = this.$refs.idMapCanvasContainer;
      const canvas = this.$refs.idMapCanvas;

      // compute the size to maintain aspect ratio
      const aspectRatio = 1;
      let width = container.clientWidth;
      let height = container.clientHeight;

      if (height > width / aspectRatio) {
        height = width / aspectRatio;
      } else {
        width = height * aspectRatio;
      }

      // set the canvas size
      canvas.width = width;
      canvas.height = height;
    },

    // draw map canvas
    drawMapCanvas: function () {
      const canvas = this.$refs.idMapCanvas;
      const ctx = canvas.getContext("2d");

      const mapInfo = this.mapData.info; // get map info from the data
      const data = this.mapData.data; // array of occupancy values

      // resize the canvas to fit the map
      canvas.width = mapInfo.width;
      canvas.height = mapInfo.height;

      // clear canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // draw the image data to the canvas
      const imageData = ctx.createImageData(mapInfo.width, mapInfo.height);
      for (let i = 0; i < mapInfo.width * mapInfo.height; i++) {
        const x = i % mapInfo.width;
        const y = Math.floor(i / mapInfo.width);
        const value = data[i];
        const color = value === 0 ? 255 : 255 - value; // map 0 (free) to white, others from white to black
        const flippedY = mapInfo.height - y - 1; // flip y-coordinate to correct orientation
        const index = (flippedY * mapInfo.width + x) * 4; // calculate the correct index for the flipped image
        imageData.data[index] = color;
        imageData.data[index + 1] = color;
        imageData.data[index + 2] = color;
        imageData.data[index + 3] = 255;
      }
      ctx.putImageData(imageData, 0, 0);

      // map origin as per canvas
      const originX = -mapInfo.origin.position.x / mapInfo.resolution;
      const originY = -mapInfo.origin.position.y / mapInfo.resolution;

      // draw the map origin
      const markerSize = 5;
      ctx.strokeStyle = "black";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(originX - markerSize, mapInfo.height - originY - markerSize);
      ctx.lineTo(originX + markerSize, mapInfo.height - originY + markerSize);
      ctx.moveTo(originX + markerSize, mapInfo.height - originY - markerSize);
      ctx.lineTo(originX - markerSize, mapInfo.height - originY + markerSize);
      ctx.stroke();

      // robot position as per canvas
      const robotX = originX + this.robotPose.x / mapInfo.resolution;
      const robotY = mapInfo.height - (originY + this.robotPose.y / mapInfo.resolution);

      // draw robot position and orientation
      const arrowLength = 20;
      const arrowThickness = 2;
      const endX = robotX + arrowLength * Math.cos(this.robotPose.yaw);
      const endY = robotY - arrowLength * Math.sin(this.robotPose.yaw);

      ctx.strokeStyle = "red";
      ctx.lineWidth = arrowThickness;
      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.lineTo(endX, endY);
      ctx.stroke();

      const headlen = 5; // length of head in pixels
      const angle = Math.atan2(endY - robotY, endX - robotX);
      ctx.beginPath();
      ctx.moveTo(endX, endY);
      ctx.lineTo(endX - headlen * Math.cos(angle - Math.PI / 6), endY - headlen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(endX - headlen * Math.cos(angle + Math.PI / 6), endY - headlen * Math.sin(angle + Math.PI / 6));
      ctx.lineTo(endX, endY);
      ctx.lineTo(endX - headlen * Math.cos(angle - Math.PI / 6), endY - headlen * Math.sin(angle - Math.PI / 6));
      ctx.stroke();
      ctx.fillStyle = "red";
      ctx.fill();

      // show map canvas
      this.showMap = true;
    },

    // clear map canvas
    clearMapCanvas: function () {
      const canvas = this.$refs.idMapCanvas;
      const ctx = canvas.getContext("2d");

      const mapInfo = this.mapData.info; // get map info from the data

      // resize the canvas to fit the map
      canvas.width = mapInfo.width;
      canvas.height = height;

      // clear canvas
      ctx.clearRect(0, 0, 0, 0);
      this.showMap = false;
    },

    // handle command request
    handleCommandRequest: function (event) {
      let command = this.commands[event.target.options.selectedIndex];
      this.controlService.serviceRequest = command.command;
      this.logMessage(0, `Command switched to "${command.name}".`);
    },

    // control service call
    callControlService: function () {
      this.controlService.isServiceBusy = true;

      // define the service client
      let service = new ROSLIB.Service({
        ros: this.ros,
        name: "/robot_command",
        serviceType: "cafeteriabot_interface/srv/RobotCommand",
      });

      // define the service request
      let request = new ROSLIB.ServiceRequest({
        command: this.controlService.serviceRequest,
      });

      // call the service and handle the response
      service.callService(
        request,
        (result) => {
          this.controlService.isServiceBusy = false;
          this.logMessage(10, `${result.message}.`);
        },
        (error) => {
          this.controlService.isServiceBusy = false;
          this.logMessage(50, `Service call failed: ${error}.`);
        },
      );
    },

    // elevator service call
    callElevatorService() {
      this.elevatorService.isServiceBusy = true;

      // define the service client
      let service = new ROSLIB.Service({
        ros: this.ros,
        name: "/elevator_control",
        serviceType: "std_srvs/srv/SetBool",
      });

      // define the service request
      let request = new ROSLIB.ServiceRequest({
        data: this.elevatorService.serviceRequest,
      });

      // call the service and handle the response
      service.callService(
        request,
        (result) => {
          // toggle the serviceRequest state for the next service call
          this.elevatorService.serviceRequest = !this.elevatorService.serviceRequest;
          this.elevatorService.isServiceBusy = false;
          this.logMessage(10, `${result.message}.`);
        },
        (error) => {
          this.elevatorService.isServiceBusy = false;
          this.logMessage(50, `Service call failed: ${error}.`);
        },
      );
    },

    // table service call
    callTableService() {
      this.tableService.isServiceBusy = true;

      // define the service client
      let service = new ROSLIB.Service({
        ros: this.ros,
        name: "/is_available",
        serviceType: "cafeteriabot_interface/srv/TableAvailable",
      });

      // define the service request
      let request = new ROSLIB.ServiceRequest({
        duration: parseFloat(this.tableService.serviceRequest),
      });

      // call the service and handle the response
      service.callService(
        request,
        (result) => {
          this.tableService.isServiceBusy = false;
          if (result.success) {
            this.showTable = true;
            this.drawTableCanvas();
            this.logMessage(10, `Table found at location (x, y) ${result.message} on map.`);
          } else {
            this.showTable = false;
            this.clearTableCanvas();
            this.logMessage(50, `${result.message}.`);
          }
        },
        (error) => {
          this.tableService.isServiceBusy = false;
          this.logMessage(50, `Service call failed: ${error}.`);
        },
      );
    },

    // send action goal (using jumper)
    sendDockGoal: function () {
      // define the service client
      let service = new ROSLIB.Service({
        ros: this.ros,
        name: "/robot_command",
        serviceType: "cafeteriabot_interface/srv/RobotCommand",
      });

      // define the service request
      let request = new ROSLIB.ServiceRequest({
        command: "dock",
      });

      // call the service and handle the response
      service.callService(
        request,
        (result) => {
          this.dockAction.status = {
            active: true,
            button: true,
          };
          this.logMessage(10, `${result.message}`);
        },
        (error) => {
          this.logMessage(50, `Action call failed: ${error}.`);
        },
      );
    },

    // cancel action goal (using jumper)
    cancelDockGoal: function () {
      // define the service client
      let service = new ROSLIB.Service({
        ros: this.ros,
        name: "/robot_command",
        serviceType: "cafeteriabot_interface/srv/RobotCommand",
      });

      // define the service request
      let request = new ROSLIB.ServiceRequest({
        command: "cancel",
      });

      // call the service and handle the response
      service.callService(
        request,
        (result) => {
          this.dockAction.status.button = false;
          this.logMessage(10, `${result.message}`);
        },
        (error) => {
          this.logMessage(50, `Action call failed: ${error}.`);
        },
      );
    },

    // reset action client state
    resetActionClient() {
      this.dockAction = {
        status: { active: false, button: false },
        feedback: { stage: 0, status: 0, success: null, message: "" },
      };
      this.progress = 0;
      this.progress_count = 0;
      this.logMessage(0, "The reset of the action client state has been successfull.");
    },

    // publisher callback to velocity
    publishVelocity: function (x, z) {
      let velocity = new ROSLIB.Topic({
        ros: this.ros,
        name: "/cleaner_2/cmd_vel",
        messageType: "geometry_msgs/msg/Twist",
      });
      let message = new ROSLIB.Message({
        linear: { x: x, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: z },
      });
      velocity.publish(message);
    },

    // joystick on page load
    joystickOnLoad() {
      // reset joystick to home position
      let ref = document.getElementById("idDragStartZone");
      let minTop = ref.offsetTop - parseInt(this.draggingStyle.height) / 2;
      let maxTop = minTop + 200;
      let top = 100.0 + minTop;
      this.draggingStyle.top = `${top}px`;

      let minLeft = ref.offsetLeft - parseInt(this.draggingStyle.width) / 2;
      let maxLeft = minLeft + 200;
      let left = 100.0 + minLeft;
      this.draggingStyle.left = `${left}px`;
    },

    // joystick on mouse click
    joystickStartDrag() {
      this.isDragging.status = true;
      this.isDragging.x = 0.0;
      this.isDragging.y = 0.0;
    },

    // joystick on mouse movement
    joystickDoDrag(event) {
      if (this.isDragging.status) {
        // calculate drag
        let ref = document.getElementById("idDragStartZone");
        this.isDragging.x = event.offsetX;
        this.isDragging.y = event.offsetY;

        let minTop = ref.offsetTop - parseInt(this.draggingStyle.height) / 2;
        let maxTop = minTop + 200;
        let top = this.isDragging.y + minTop;
        this.draggingStyle.top = `${top}px`;

        let minLeft = ref.offsetLeft - parseInt(this.draggingStyle.width) / 2;
        let maxLeft = minLeft + 200;
        let left = this.isDragging.x + minLeft;
        this.draggingStyle.left = `${left}px`;

        // update joystick values
        this.joystick.x = -1 * (this.isDragging.y / 200 - 0.5);
        this.joystick.z = -1 * (this.isDragging.x / 200 - 0.5);

        // publish on topic
        this.publishVelocity(this.joystick.x, this.joystick.z);
      }
    },

    // joystick on mouse release
    joystickStopDrag() {
      this.isDragging.status = false;
      this.isDragging.x = this.isDragging.y = "no";

      // reset joystick to home position
      let ref = document.getElementById("idDragStartZone");
      let minTop = ref.offsetTop - parseInt(this.draggingStyle.height) / 2;
      let maxTop = minTop + 200;
      let top = 100.0 + minTop;
      this.draggingStyle.top = `${top}px`;

      let minLeft = ref.offsetLeft - parseInt(this.draggingStyle.width) / 2;
      let maxLeft = minLeft + 200;
      let left = 100.0 + minLeft;
      this.draggingStyle.left = `${left}px`;

      // update joystick values
      this.joystick.x = 0.0;
      this.joystick.z = 0.0;

      // publish on topic
      this.publishVelocity(this.joystick.x, this.joystick.z);
    },

    // ui cleanup
    cleanUp: function () {
      this.clearTableCanvas();
      this.clearMapCanvas();
      this.resetActionClient();
    },
  },

  mounted() {
    // load joystick on page load
    this.joystickOnLoad();

    // load table canvas on page load
    this.adjustTableCanvasSize();
    this.drawTableCanvas();

    // update map in certain interval
    this.interval = setInterval(() => {
      if (this.ros != null && this.ros.isConnected) {
        this.ros.getNodes(
          (data) => {},
          (error) => {},
        );
      }
    }, 10000);

    // reset joystick on mouse release
    window.addEventListener("mouseup", this.joystickStopDrag);
  },
});
