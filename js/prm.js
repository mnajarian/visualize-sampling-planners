Array.prototype.remove = function(from, to) {
  var rest = this.slice((to || from) + 1 || this.length);
  this.length = from < 0 ? this.length + from : from;
  return this.push.apply(this, rest)
};
Array.prototype.contains = function(element) {
  for (var i = 0; i < this.length; i++)
    if (this[i] == element) return true
};

function angle(a, b) {
  var x = b[1] - a[1];
  var y = b[0] - a[0];
  if (x == 0) Math.PI / 2;
  return Math.atan(y / x)
}

function isLeftTurn(a, b, c) {
  var product = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
  return product <= 0
}

function triangleContainsPoint(a, b, c, p) {
  function crossProduct(p, q) {
    return p[0] * q[1] - p[1] * q[0]
  }

  function dotProduct(p, q) {
    return p * q
  }

  function minus(p, q) {
    return [p[0] - q[0], p[1] - q[1]]
  }

  function sameSide(p1, p2, a, b) {
    cp1 = crossProduct(minus(b, a), minus(p1, a));
    cp2 = crossProduct(minus(b, a), minus(p2, a));
    return dotProduct(cp1, cp2) >= 0
  }
  return sameSide(p, a, b, c) && sameSide(p, b, a, c) && sameSide(p, c, a, b)
}

function distance(one, two) {
  return Math.pow(Math.pow(one[0] - two[0], 2) + Math.pow(one[1] - two[1], 2), 0.5)
}

function triangulate(points, raph) {
  var bottom = 0;
  for (var i = 0; i < points.length; i++)
    if (points[i][1] > points[bottom][1]) bottom = i;
  dir = 1;
  var bp = points[bottom];
  var bpr = points[(bottom + 1) % points.length];
  var bpl = points[(bottom - 1 + points.length) % points.length];
  if (angle(bp, bpr) > angle(bp, bpl)) dir = -1;
  var triangles = [];
  while (points.length > 3) {
    var before = points.length;
    for (var i = 0; i < points.length; i++) {
      var j = (i + 1 * dir + points.length) % points.length;
      var k = (i + 2 * dir + points.length) % points.length;
      var a = points[i];
      var b = points[j];
      var c = points[k];
      var ear = isLeftTurn(a, b, c);
      for (var l = 0; l < points.length; l++)
        if (l != i && l != j && l != k)
          if (triangleContainsPoint(a, b, c, points[l])) ear = false;
      var t = "ear";
      if (!ear) t = "no";
      if (ear) {
        triangles.push([
          [a[0], a[1]],
          [b[0], b[1]],
          [c[0], c[1]]
        ]);
        points.remove(j)
      }
    }
    if (before == points.length && before != 3) {
      break
    }
    before = points.length
  }
  if (points.length == 3) triangles.push(points);
  return triangles
}

function segmentsIntersect(p, q, a, b) {
  function crossProduct(p, q) {
    return p[0] * q[1] - p[1] * q[0]
  }

  function dotProduct(p, q) {
    return p * q
  }

  function minus(p, q) {
    return [p[0] - q[0], p[1] - q[1]]
  }

  function sameSide(p1, p2, a, b) {
    cp1 = crossProduct(minus(b, a), minus(p1, a));
    cp2 = crossProduct(minus(b, a), minus(p2, a));
    return dotProduct(cp1, cp2) >= 0
  }
  return !sameSide(p, q, a, b) && !sameSide(a, b, p, q)
}

function link(p, q, a, b, c) {
  return !segmentsIntersect(p, q, a, b) && !segmentsIntersect(p, q, b, c) && !segmentsIntersect(p, q, c, a)
}

function drawPathFromPoints(points, raph) {
  var p = "M" + points[0][0] + "," + points[0][1];
  for (var i = 1; i < points.length; i++) p += "L" + points[i][0] + "," + points[i][1];
  p += "L" + points[0][0] + "," + points[0][1];
  return raph.path(p)
}

function beginDraw(color, r, x, y) {
  var points = [];
  var dot;
  var segments = [];

  function addPoint(x, y) {
    if (points.length > 0 && x == points[points.length - 1][0] && y == points[points.length - 1][1]) return 0;
    points.push([x, y]);
    if (points.length == 1) dot = r.circle(x, y, 3);
    else {
      var x0 = points[points.length - 2][0];
      var y0 = points[points.length - 2][1];
      var x1 = points[points.length - 1][0];
      var y1 = points[points.length - 1][1];
      var newpath = r.path("M" + x0 + "," + y0 + "L" + x1 + "," + y1);
      segments.push(newpath)
    }
    if (points.length >= 3 && distance(points[0], points[points.length -
        1]) < 10) {
      points[points.length - 1][0] = points[0][0];
      points[points.length - 1][1] = points[0][1];
      for (var i = 0; i < segments.length; i++) segments[i].remove();
      var pathstring = "M" + points[0][0] + "," + points[0][1];
      for (var i = 1; i < points.length; i++) pathstring += "L" + points[i][0] + "," + points[i][1];
      dot.remove();
      points.remove(-1);
      r.path(pathstring).attr("fill", color).toBack();
      return points
    }
    return 0
  }
  addPoint(x, y);
  return addPoint
}

function newRandomNodes(obstacles) {
  var numNodes = parseInt($("#samples").val());
  var nodes = [];
  var triangles = triangulateAll(obstacles);
  for (var i = 0; i < numNodes; i++) nodes.push(getNewRandomNode(triangles));  
  return nodes;
}

function steer(node1, node2) {
  // return a node at the midpoint between node1 and node2
  var steerNode = [];
  var steerX = (node1[0] + node2[0])/2.0;
  var steerY = (node1[1] + node2[1])/2.0; 
  steerNode[0] = steerX;
  steerNode[1] = steerY;
  return steerNode
}

function obstacleFree(node, triangles) {
  var clear = true;
  for (var i = 0; i < triangles.length; i++) {
    var o = triangles[i];
    if (triangleContainsPoint(o[0], o[1], o[2], node)) clear = false
  }
  return clear
}


function computeRoadmapRRT(roadmap, obstacles, newNodes, variant, startNode) {
  var addNewNodes = newNodes.slice();
  var nodes = [startNode];
  var triangles = triangulateAll(obstacles);
  var adjacencyList = [];
  adjacencyList.push([]);
  var costs = [0];
  var parents = [0];
  if (roadmap != null){
    nodes = roadmap["nodes"];
    adjacencyList = roadmap["adjacencyList"];
    costs = roadmap["costs"];
    parents = roadmap["parents"];
  }
  for (var i = nodes.length - newNodes.length; i < nodes.length; i++) {
    var l = [];
    adjacencyList.push(l);
    costs.push(null);
    parents.push(null);
  }
  while (addNewNodes.length > 0){
    var newFreeNode = null;
    var closestNodeDistance = Infinity;
    var closestNode = null;
    // get new node to add based on closest distance to exising nodes
    for (var k=0; k < addNewNodes.length; k++){
      for (var j=0; j < nodes.length; j++){
        if (distance(nodes[j], addNewNodes[k]) < closestNodeDistance) {
          newFreeNode = addNewNodes[k]; 
          closestNode = nodes[j];
          closestNodeDistance = distance(nodes[j], addNewNodes[k]);
        }
      }
    }
    var newNode = steer(closestNode, newFreeNode); 
    if (obstacleFree(newNode, triangles)) {
      if (variant == "rrt") {
        if (linkExists(closestNode, newNode, triangles) && closestNode != newNode) {
          nodes.push(newNode);
          var closestNodeIndex = nodes.indexOf(closestNode);
          var newNodeIndex = nodes.indexOf(newNode);
          adjacencyList[closestNodeIndex].push(newNodeIndex);
          //adjacencyList[newNodeIndex] = [closestNodeIndex];
          adjacencyList[newNodeIndex].push(closestNodeIndex);
        }
      }
      else if (variant == "rrt-star") {
        var xMin = closestNode;
        var cMin = costs[nodes.indexOf(xMin)] + distance(xMin, newNode);

        // get closest existing nodes surrounding newNode
        var closestNodeIndices = [];
        for (var t=0; t < nodes.length; t++){
          if (distance(nodes[t], newNode) < 700.0) closestNodeIndices.push(t)
        }
        nodes.push(newNode);
        for (var i=0; i < closestNodeIndices.length; i++) {
          var xNearIndex = closestNodeIndices[i];
          var xNear = nodes[xNearIndex];
          if ((linkExists(xNear, newNode, triangles)) &&
             (linkExists(newNode, xNear, triangles)) &&  
             (costs[xNearIndex] + distance(nodes[xNearIndex], newNode) < cMin)){
            console.log(triangles);
            console.log(linkExists(xNear, newNode, triangles));
            xMin = xNear;
            cMin = costs[xNearIndex] + distance(nodes[xNearIndex], newNode); 
          } 
        }
        var newNodeIndex = nodes.indexOf(newNode);
        var xMinIndex = nodes.indexOf(xMin);
        adjacencyList[xMinIndex].push(newNodeIndex);
        adjacencyList[newNodeIndex].push(xMinIndex);
        costs[newNodeIndex] = costs[xMinIndex] + distance(newNode, nodes[xMinIndex]);
        parents[newNodeIndex] = xMinIndex;       
 
        for (var m=0; m < closestNodeIndices.length; m++) {
          var xNear = closestNodeIndices[m];
          if (linkExists(newNode, nodes[xNear], triangles) && 
             (costs[newNodeIndex] + distance(newNode, nodes[xNear]) < costs[xNear])){
            var xParent = parents[xNear];
            // Remove (xParent, xNear)
            adjacencyList[xParent].splice(adjacencyList[xParent].indexOf(xNear), 1);
            adjacencyList[xNear].splice(adjacencyList[xNear].indexOf(xParent), 1);
            // Add (newNode, xNear)
            adjacencyList[xNear].push(newNodeIndex);
            adjacencyList[newNodeIndex].push(xNear);
            costs[xNear] = costs[newNodeIndex] + distance(newNode, nodes[xNear]);
            parents[xNear] = newNodeIndex;
          }
        }
      }
    }
    var i = addNewNodes.indexOf(newFreeNode);
    if (i != -1) addNewNodes.splice(i, 1)
  }  
  return {
    "nodes": nodes,
    "adjacencyList": adjacencyList,
    "costs": costs,
    "parents": parents
  }
}

function computeRoadmapPRM(roadmap, obstacles, newNodes, variant) {
  // extends an existing roadmap with given obstacles, 
  // based on the variant of sampling-based motion planner

  var nodes = [];
  if (roadmap != null) nodes = roadmap["nodes"];
  var numNeighbors = null;
  if (variant == "k-prm"){
    numNeighbors = parseInt($("#neighbors").val());
  } else if (variant == "k-prm-star"){
    if (nodes.length > 0){
      numNeighbors = 2 * Math.E * (Math.log(nodes.length));
    } else {
      numNeighbors = parseInt($("#neighbors").val());
    }
  }
  var triangles = triangulateAll(obstacles);
  for (var i=0; i < newNodes.length; i++) nodes.push(newNodes[i]);
  var adjacencyList = [];
  if (roadmap != null) adjacencyList = roadmap["adjacencyList"];
  for (var i = nodes.length - newNodes.length; i < nodes.length; i++) {
    var l = [];
    for (var j = 0; j < nodes.length; j++)
      if (linkExists(nodes[i],
          nodes[j], triangles) && i != j) l.push(j);
    l.sort(function(l, r) {
      return distance(nodes[i], nodes[l]) - distance(nodes[i], nodes[r])
    });
    while (l.length > numNeighbors) l.remove(l.length - 1);
    adjacencyList.push(l)
  }
  for (var i = 0; i < nodes.length; i++)
    for (var j = 0; j < nodes.length; j++) {
      var li = adjacencyList[i];
      var lj = adjacencyList[j];
      if (li.contains(j)) {
        if (!lj.contains(i)) lj.push(i)
      } else if (lj.contains(i))
        if (!li.contains(j)) li.push(j)
    }
  return {
    "nodes": nodes,
    "adjacencyList": adjacencyList
  }
}

function drawRoadmap(roadmap, r) {
  console.log(roadmap);
  var drawn = [];
  var nodes = roadmap["nodes"];
  for (var i = 0; i < nodes.length; i++) {
    var p = nodes[i];
    drawn.push(r.circle(p[0], p[1], 2).attr("stroke", "#32cd32"));
    var list = roadmap["adjacencyList"][i];
    for (var j = 0; j < list.length; j++) {
      var dest = nodes[list[j]];
      var pathString = "M" + p[0] + "," + p[1] + "L" + dest[0] + "," + dest[1];
      drawn.push(r.path(pathString).attr("stroke", "#008080"))
    }
  }
  return drawn
}

function getNewRandomNode(triangles) {
  var maxAttempts = 5E3;
  var count = 0;
  while (true || count >= maxAttempts) {
    count += 1;
    var x = 2 + Math.random() * 344;
    var y = 2 + Math.random() * 498;
    if (obstacleFree([x,y], triangles)) return [x,y]
  }
}

function triangulateAll(obstacles) {
  var triangles = [];
  for (var i = 0; i < obstacles.length; i++) {
    var ob = obstacles[i];
    var tris = triangulate(copyObstacles(ob));
    for (var j = 0; j < tris.length; j++) triangles.push(tris[j])
  }
  return triangles
}

function linkExists(p, q, triangles) {
  for (var i = 0; i < triangles.length; i++) {
    var t = triangles[i];
    if (!link(p, q, t[0], t[1], t[2])) return false
  }
  return true
}

function copyObstacles(arr) {
  var a = [];
  for (var i = 0; i < arr.length; i++) a.push([arr[i][0], arr[i][1]]);
  return a
}

function calcShortestPath(start, goal, roadmap) {
  var adj = roadmap["adjacencyList"];
  var nodes = roadmap["nodes"];
  var snode = null;
  var smin = 1E8;
  var gnode = null;
  var gmin = 1E8;
  for (var i = 0; i < nodes.length; i++) {
    var sdist = distance(start, nodes[i]);
    if (sdist < smin) {
      smin = sdist;
      snode = i
    }
    var gdist = distance(goal, nodes[i]);
    if (gdist < gmin) {
      gmin = gdist;
      gnode = i
    }
  }
  var shortest = [];
  var previous = [];
  var q = [];
  for (var i = 0; i < nodes.length; i++) {
    shortest.push(1E11);
    previous.push(-1);
    q.push(i)
  }
  shortest[snode] = 0;
  while (q.length > 0) {
    var u = -1;
    var ui = -1;
    var min = 1E11;
    for (var j = 0; j < q.length; j++) {
      var cand = q[j];
      if (shortest[cand] < min) {
        min = shortest[cand];
        ui = j;
        u = cand
      }
    }
    if (shortest[u] == 1E11 || u == -1) break;
    q.remove(ui);
    for (var i = 0; i < adj[u].length; i++) {
      var adjacentNode = adj[u][i];
      var alt = shortest[u] + distance(nodes[u], nodes[adjacentNode]);
      if (alt < shortest[adjacentNode]) {
        shortest[adjacentNode] = alt;
        previous[adjacentNode] = u
      }
    }
  }
  var path = [];
  var curr = gnode;
  while (true) {
    path.push(nodes[curr]);
    curr = previous[curr];
    if (curr == snode || curr == null) break
  }
  path.push(nodes[snode]);
  if (path[1] == null){
    return null;
  } else {
    return path;
  } 
}

function drawPath(goal, path, start, r) {
  var p = "M" + goal[0] + "," + goal[1] + "L" + path[0][0] + "," + path[0][1];
  for (var i = 0; i < path.length; i++) p += "L" + path[i][0] + "," + path[i][1];
  p += "L" + start[0] + "," + start[1];
  var path = r.path(p);
  path.attr("stroke", "#ff0065").attr("stroke-width", 2);
  return path
}


$(function() {
  var r1 = Raphael("draw-1", 348, 500);
  var r2 = Raphael("draw-2", 348, 500);
  var drawingState = "none";
  var update;
  var roadmap1 = null;
  var roadmap2 = null;
  var drawnRoadmap1 = [];
  var drawnRoadmap2 = [];
  var obstacles = [];
  var start = [100, 450];
  var goal = [300, 100];
  var drawnPath1 = null;
  var drawnPath2 = null;
  var panel1Variant = "k-prm";
  var panel2Variant = "k-prm-star";
  $("#instructions").html("Click to draw obstacles");

  function drawStartAndGoal() {
    r1.circle(start[0], start[1], 4).attr("fill", "#ffff00");
    r1.circle(goal[0], goal[1], 4).attr("fill", "#ffc0cb");
    r2.circle(start[0], start[1], 4).attr("fill", "#ffff00");
    r2.circle(goal[0], goal[1], 4).attr("fill", "#ffc0cb");
  }
  drawStartAndGoal();

  function drawObstacle(r, points, color){
    var pathstring = "M" + points[0][0] + "," + points[0][1];
    for (var i = 1; i < points.length; i++) pathstring += "L" + points[i][0] + "," + points[i][1];
    pathstring += "Z";
    r.path(pathstring).attr("fill", color).toBack(); 
  };

  function clearAll(){
    $("#instructions").html("Click to draw obstacles");
    r1.clear();
    r2.clear();
    sums = [];
    obstacles = [];
    opaths = [];
    robot = null;
    dot = null;
    drawnPath1 = null;
    drawPath2 = null;
    roadmap1 = null;
    roadmap2 = null;
    drawnRoadmap1 = [];
    drawnRoadmap2 = [];
    drawingState = "none";
    drawStartAndGoal();
  }

  $("#sampler-selection").change(function() {
    if ($(this).val() == "rrt"){
      panel1Variant = "rrt";
      panel2Variant = "rrt-star";
      $("#neighbors-input").hide();
    }
    else if ($(this).val() == "k-prm"){
      panel1Variant = "k-prm";
      panel2Variant = "k-prm-star";
      $("#neighbors-input").show();
    }
    clearAll();
  });

  $("#draw-1").click(function(e) {
    if (drawingState == "none") {
      update = beginDraw("#f00", r1, e.offsetX, e.offsetY);
      drawingState = "drawing"
    } else if (drawingState == "drawing") {
      var result = update(e.offsetX, e.offsetY);
      if (result != 0) {
        obstacles.push(result);
        drawingState = "none";
        drawObstacle(r2, result, "#ff0000")
      }
    }
  });

  $("#draw-2").click(function(e) {
    if (drawingState == "none") {
      update = beginDraw("#f00", r2, e.offsetX, e.offsetY);
      drawingState = "drawing"
    } else if (drawingState == "drawing") {
      var result = update(e.offsetX, e.offsetY);
      if (result != 0) {
        obstacles.push(result);
        drawingState = "none";
        drawObstacle(r1, result, "#ff0000")
      }
    }
  });

  $("#calc").click(function() {
    $("#instructions").html("Clear roadmap to draw");
    var newNodes = newRandomNodes(obstacles);
    if ($("#sampler-selection").val() == "k-prm"){
      roadmap1 = computeRoadmapPRM(roadmap1, obstacles, newNodes, panel1Variant);
      roadmap2 = computeRoadmapPRM(roadmap2, obstacles, newNodes, panel2Variant);
    } else if ($("#sampler-selection").val() == "rrt"){
      roadmap1 = computeRoadmapRRT(roadmap1, obstacles, newNodes, panel1Variant, start);
      roadmap2 = computeRoadmapRRT(roadmap2, obstacles, newNodes, panel2Variant, start);
    }

    for (var i = 0; i < drawnRoadmap1.length; i++) drawnRoadmap1[i].remove();
    drawnRoadmap1 = drawRoadmap(roadmap1, r1);
    drawingState = "disabled"
    if (drawnPath1 != null) drawnPath1.remove();
    if (roadmap1 == null) return;
    var path1 = calcShortestPath(start, goal, roadmap1);
    if (path1 != null){
        drawnPath1 = drawPath(goal, path1, start, r1);
    }

    for (var i = 0; i < drawnRoadmap2.length; i++) drawnRoadmap2[i].remove();
    drawnRoadmap2 = drawRoadmap(roadmap2, r2);
    drawingState = "disabled"
    if (drawnPath2 != null) drawnPath2.remove();
    if (roadmap2 == null) return;
    var path2 = calcShortestPath(start, goal, roadmap2);
    if (path2 != null){
      drawnPath2 = drawPath(goal, path2, start, r2);
    }
  });

  $("#clear").click(function() {
    clearAll();
  });

  $("#clearroadmap").click(function() {
    $("#instructions").html("Click to draw obstacles");
    if (drawnPath1 != null) drawnPath1.remove();
    drawnPath1 = null;
    roadmap1 = null;
    if (drawnRoadmap1 != [])
      for (var i = 0; i < drawnRoadmap1.length; i++) drawnRoadmap1[i].remove();
    drawingState = "none";
    drawnRoadmap1 = [];

    if (drawnPath2 != null) drawnPath2.remove();
    drawnPath2 = null;
    roadmap2 = null;
    if (drawnRoadmap2 != [])
      for (var i = 0; i < drawnRoadmap2.length; i++) drawnRoadmap2[i].remove();
    drawingState = "none";
    drawnRoadmap2 = [];
  })
});
