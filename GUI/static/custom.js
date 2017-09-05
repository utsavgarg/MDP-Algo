document.addEventListener('DOMContentLoaded', function initialize(e) {

	// Getting elements of arena
	var canvas = document.getElementById('canvas');
	var context = canvas.getContext('2d');
	const ROWS = 20, COLUMNS = 15;
	var roboPath = [];

	// Initializing arena to be unexplored
	var map = [];
	for (var i = 0; i < ROWS; i++){
		map[i] = [];
		for (var j = 0; j < COLUMNS; j++){
			map[i][j] = 0;
		}
	}

	function getStyle(cell) {
	    switch(cell) {
	        case 0: return "#1a1e24"; // unexplored
	        case 1: return "#d3d3d3"; // explored
	        case 2: return "#f44336"; // obstacle
	        case 3: return "#03a9f4"; // start
	        case 4: return "#009688"; // goal
	        case 5: return "#428BCA"; // robot
	        case 6: return "#ffeb3b"; // path
	        case 7: return "#ff9800"; // fastest path
	        case 8: return "#673ab7"; // waypoint
	        default: return "#1a1e24";
	    }
	}

	function draw(map, center=null, head=null) {
        context.save();
        context.strokeStyle = "#252a33";
        context.lineWidth = 3;

        // Filling explored and unexplored cells
        for (var i = 0; i < ROWS; i++){
			for (var j = 0; j < COLUMNS; j++){
				context.beginPath();
				context.fillStyle = getStyle(map[i][j]);
				context.rect(30 * j, 30 * i, 30, 30);
				context.fill();
				context.stroke();
				context.closePath();
			}
		}
		context.fill();

		// add path
		if (roboPath){
			for(var i=0; i<roboPath.length; i++){
				pt = roboPath[i].split(', ');
				pt = [parseInt(pt[0]), parseInt(pt[1])];
				context.beginPath();
				context.fillStyle = getStyle(6);
				context.rect(30 * pt[1], 30 * pt[0], 30, 30);
				context.fill();
				context.stroke();
				context.closePath();
			}
		}

		// marking robot
		if (center && head){
			center = center.split(', ');
			head = head.split(', ');
			center = [parseInt(center[0]), parseInt(center[1])];
			head = [parseInt(head[0]), parseInt(head[1])];
			context.beginPath();
			context.fillStyle = getStyle(5);
			context.moveTo(30*center[1] + 55, 30*center[0] + 15);
			context.arc(30*center[1] + 15, 30*center[0] + 15, 40, 0, 2 * Math.PI, true);
			context.fill();
			context.stroke();
			context.closePath();
			context.beginPath();
			context.moveTo(30*head[1] + 20, 30*head[0] + 20);
			context.arc(30*head[1] + 15, 30*head[0] + 20, 5, 0, 2 * Math.PI, true);
			context.stroke();
			context.closePath();
		}
        context.restore();
    }


	document.getElementById('start').addEventListener('click', function(e){
		var r = new XMLHttpRequest();
		r.open("GET", "/start");
		r.send();
	});

	document.getElementById('reset').addEventListener('click', function(e){
		var r = new XMLHttpRequest();
		r.open("GET", "/reset");
		r.send();
	});

	document.getElementById('stop').addEventListener('click', function(e){
		var r = new XMLHttpRequest();
		r.open("GET", "/stop");
		r.send();
	});

	function parseJson(map){
		var parsedMap = [];
		map = map.slice(1, -1); // Removing extra brackets
		var tempCnt = 1; // For removing extra spaces
		for (i = 0; i < 20; i++){
			var row = map.slice(i*30+tempCnt, i*30+tempCnt + 30 - 1);
			tempCnt += 3;
			parsedMap.push([]);
			for (j = 0; j < 30; j+=2){
				parsedMap[i].push(parseInt(row[j]));
			}
		}
		return parsedMap;
	}

	function waypoint(){
		document.getElementById('waypoint').style.display = 'block';
	}

	function fastestpath(){
		document.getElementById('waypoint').style.display = 'none';
	}

	function wsConnect() {
		ws = new WebSocket("ws://"+window.location.host+"/websocket?Id=" + Math.floor(Math.random() * 100));
	    ws.onopen = function() {
	        ws.send("Initializing connection");
	    };
	    ws.onmessage = function (evt) {
	    	var data = JSON.parse(evt.data);
	    	var map = parseJson(JSON.parse(data.map));
	    	var center = data.center;
	    	var head = data.head;
	    	roboPath.push(center);
	    	draw(map, center, head);

	    };
	}

	draw(map);
	wsConnect();

});
