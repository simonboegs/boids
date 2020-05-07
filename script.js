var canvas = document.getElementById("world");
var ctx = canvas.getContext("2d");
var width = 600;
var height = 600;
var frameRate = 60;

async function update(state) {
    var range = document.getElementById("rangeInput").value;
    var angleOfSight = (3 / 4) * Math.PI;
    // var repelConst = 2;
    // var cohesionConst = 0.1;
    // var velMatchConst = 0.5;
    var repelConst = document.getElementById("repelConstInput").value / 100;
    var fearConst = 3;
    var avoidConst = 1;
    var cohesionConst =
        document.getElementById("cohesionConstInput").value / 100;
    var velMatchConst =
        document.getElementById("velMatchConstInput").value / 100;
    var alignmentConst = 0.1;
    var border = 100;
    var proximity = 10;
    var avoidSteerConst = 0.1;
    var goalSeekConst =
        document.getElementById("goalSeekConstInput").value / 100;
    //var minSpeed = document.getElementById("minSpeedInput");
    var minSpeed = document.getElementById("minSpeedInput").value / 100;
    var maxSpeed = document.getElementById("maxSpeedInput");
    boids = state.boids;
    await boids.forEach(async (boid) => {
        var localBoids = await getLocalBoids(boid, boids, range, angleOfSight);
        var localObstacles = getLocalObstacles(
            boid,
            state.obstacles,
            range,
            proximity
        );
        // var localPredators = getLocalPredators(boid, state.predators, range);

        if (state.goalSeeking.bool) {
            var goalSeek = getGoalSeek(
                boid,
                state.goalSeeking.pos,
                goalSeekConst
            );
            boid.vel[0] += goalSeek[0];
            boid.vel[1] += goalSeek[1];
        }

        if (localBoids.length > 0) {
            var repel = await getRepel(boid, localBoids, repelConst);
            var cohesion = await getCohesion(boid, localBoids, cohesionConst);
            var velMatch = await velocityMatch(boid, localBoids, velMatchConst);
            var alignment = getAlignment(boid, localBoids, alignmentConst);
            boid.vel[0] += repel[0] + cohesion[0] + velMatch[0];
            boid.vel[1] += repel[1] + cohesion[1] + velMatch[1];
        }
        // if (localPredators.length > 0) {
        //     var fear = await getRepel(boid, localPredators, fearConst);
        //     boid.vel[0] += fear[0];
        //     boid.vel[1] += fear[1];
        // }
        if (localObstacles.length > 0) {
            // var avoid = getAvoid(boid, localObstacles, proximity);
            // boid.vel[0] += avoid[0];
            // boid.vel[1] += avoid[1];
            var avoidSteer = await getAvoidSteer(
                boid,
                localObstacles,
                proximity,
                avoidSteerConst
            );
            console.log(avoidSteer);
            boid.vel[0] += avoidSteer[0];
            boid.vel[1] += avoidSteer[1];
        }
        boid.vel = [
            Math.round((boid.vel[0] + Number.EPSILON) * 100) / 100,
            Math.round((boid.vel[1] + Number.EPSILON) * 100) / 100,
        ];
        if (!(boid.vel[0] == 0 && boid.vel[1] == 0))
            boid.direction = getDirection(boid.vel);
        var velMag = distance([0, 0], boid.vel);
        if (velMag < minSpeed)
            boid.vel = convert(minSpeed, boid.direction, [0, 0]);
        // else if (velMag > maxSpeed)
        //     boid.vel = convert(maxSpeed, boid.direction, [0, 0]);
        //boid.vel = [+boid.vel[0].toFixed(2), +boid.vel[1].toFixed(2)];
    });
    //await predators.forEach((pred) => {});
    await boids.forEach((boid) => {
        boid.pos[0] += boid.vel[0];
        boid.pos[1] += boid.vel[1];

        boid.pos = checkPos(boid.pos, width, height, border);
        // if (boid.pos[0] < -border) boid.pos[0] += 2 * border + width;
        // else if (boid.pos[0] > width + border)
        //     boid.pos[0] = boid.pos[0] - 2 * border - width;
        // if (boid.pos[1] < -border) boid.pos[1] += 2 * border + height;
        // else if (boid.pos[1] > height + border)
        //     boid.pos[1] = boid.pos[1] - 2 * border - height;
    });
    state.predators.forEach((pred) => {
        pred.pos[0] += pred.vel[0];
        pred.pos[1] += pred.vel[1];
        pred.pos = checkPos(pred.pos, width, height, border);
    });
    return state;
}

async function render(state) {
    ctx.fillStyle = "#545454";
    await ctx.fillRect(0, 0, width, height);
    state.obstacles.forEach((obs) => {
        ctx.beginPath();
        ctx.arc(obs.pos[0], obs.pos[1], obs.radius, 0, 2 * Math.PI);
        ctx.strokeStyle = obs.color;
        ctx.stroke();
    });
    state.boids.forEach(async (boid) => {
        var triCords = await getTriCords(boid);
        ctx.beginPath();
        ctx.moveTo(Math.round(triCords[0][0]), Math.round(triCords[0][1]));
        ctx.lineTo(Math.round(triCords[1][0]), Math.round(triCords[1][1]));
        ctx.lineTo(Math.round(triCords[2][0]), Math.round(triCords[2][1]));
        ctx.closePath();
        ctx.lineWidth = 1;
        ctx.strokeStyle = boid.color;
        ctx.stroke();
    });
    state.predators.forEach(async (pred) => {
        var triCords = await getTriCords(pred);
        ctx.beginPath();
        ctx.moveTo(Math.round(triCords[0][0]), Math.round(triCords[0][1]));
        ctx.lineTo(Math.round(triCords[1][0]), Math.round(triCords[1][1]));
        ctx.lineTo(Math.round(triCords[2][0]), Math.round(triCords[2][1]));
        ctx.closePath();
        ctx.lineWidth = 1;
        ctx.strokeStyle = pred.color;
        ctx.stroke();
    });
}

async function getTriCords(boid) {
    var dist = Math.sqrt(
        Math.pow(boid.width / 2, 2) + Math.pow(boid.height / 2, 2)
    );
    var A = Math.atan(boid.width / boid.height);
    var angle = Math.PI - A;
    var point1 = convert(dist, checkAngle(boid.direction + angle), boid.pos);
    var point2 = convert(dist, checkAngle(boid.direction - angle), boid.pos);
    var head = convert(boid.height / 2, boid.direction, boid.pos);
    return [head, point1, point2];
}

function convert(r, theta, orgin) {
    var x = r * Math.cos(theta);
    var y = r * Math.sin(theta);
    return [x + orgin[0], y + orgin[1]];
}

function checkAngle(angle) {
    if (angle >= 2 * Math.PI) return angle - 2 * Math.PI;
    if (angle < 0) return angle + 2 * Math.PI;
    return angle;
}

function checkPos(pos, width, height, border) {
    newPos = [pos[0], pos[1]];
    if (pos[0] < -border) newPos[0] += 2 * border + width;
    else if (pos[0] > width + border)
        newPos[0] = newPos[0] - 2 * border - width;
    if (pos[1] < -border) newPos[1] += 2 * border + height;
    else if (pos[1] > height + border)
        newPos[1] = newPos[1] - 2 * border - height;
    return newPos;
}

function createBoid(pos, vel, color) {
    const obj = {};
    obj.pos = pos;
    obj.vel = vel;
    obj.color = color;
    obj.width = 3;
    obj.height = 6;
    obj.direction = getDirection(vel);
    return obj;
}

function createObstacle(pos, radius, color) {
    const obj = {};
    obj.pos = pos;
    obj.radius = radius;
    obj.color = color;
    return obj;
}

function createPredator(pos, vel, color) {
    const obj = {};
    obj.pos = pos;
    obj.vel = vel;
    obj.color = color;
    obj.width = 10;
    obj.height = 20;
    obj.direction = getDirection(vel);
    return obj;
}

function getDirection(vel) {
    a = Math.abs(Math.atan(vel[1] / vel[0]));
    if (vel[0] >= 0 && vel[1] >= 0) return a;
    else if (vel[0] < 0 && vel[1] >= 0) return Math.PI - a;
    else if (vel[0] <= 0 && vel[1] < 0) return Math.PI + a;
    else if (vel[0] > 0 && vel[1] < 0) return 2 * Math.PI - a;
}

async function getLocalBoids(boid, boids, range, angleOfSight) {
    var localBoids = [];
    await boids.forEach((b) => {
        var dist = Math.sqrt(
            Math.pow(boid.pos[0] - b.pos[0], 2) +
                Math.pow(boid.pos[1] - b.pos[1], 2)
        );
        if (dist <= range && b != boid) {
            localBoids.push(b);
            // var angle = Math.abs(boid.direction - getAngle(boid.pos, b.pos));
            // if (angle <= angleOfSight) localBoids.push(b);
        }
    });
    return localBoids;
}

function getLocalObstacles(boid, obstacles, range, proximity) {
    var localObs = [];
    obstacles.forEach((obs) => {
        // var dist =
        //     Math.sqrt(
        //         Math.pow(boid.pos[0] - obs.pos[0], 2) +
        //             Math.pow(boid.pos[1] - obs.pos[1], 2)
        //     ) - obs.radius;
        var dist = distance(boid.pos, obs.pos) - obs.radius - proximity;
        if (dist < 0) state.boids.splice(state.boids.indexOf(boid), 1);
        // if (dist <= range) {
        //     localObs.push(obs);
        //     console.log(dist);
        // }
        if (dist <= range) {
            var angleToObs = getAngle(boid.pos, obs.pos);
            var littleAngle = Math.atan(
                obs.radius + proximity / (dist + obs.radius + proximity)
            );
            if (
                Math.min(angleToObs - littleAngle, angleToObs + littleAngle) <
                    boid.direction &&
                boid.direction <
                    Math.max(angleToObs - littleAngle, angleToObs + littleAngle)
            )
                localObs.push(obs);
        }
    });
    return localObs;
}

function getLocalPredators(boid, predators, range) {
    localPreds = [];
    predators.forEach((pred) => {
        if (distance(boid.pos, pred.pos) <= range + 100) localPreds.push(pred);
    });
    return localPreds;
}

async function getRepel(boid, localBoids, repelConst) {
    var repelX = 0;
    var repelY = 0;
    await localBoids.forEach((b) => {
        var repelForce = repelConst / distance(boid.pos, b.pos);
        var repelDirection = getAngle(b.pos, boid.pos);
        var repel = convert(repelForce, repelDirection, [0, 0]);
        repelX += repel[0];
        repelY += repel[1];
    });
    return [repelX, repelY];
}

async function getCohesion(boid, localBoids, cohesionConst) {
    var avgPos = getAvgPosition(localBoids);
    var diff = [avgPos[0] - boid.pos[0], avgPos[1] - boid.pos[1]];
    var unitVector = getUnitVector(diff);
    return [unitVector[0] * cohesionConst, unitVector[1] * cohesionConst];
}

function velocityMatch(boid, localBoids, velMatchConst) {
    var avgVel = getAvgVelocity(localBoids);
    var diff = [avgVel[0] - boid.vel[0], avgVel[1] - boid.vel[1]];
    return [diff[0] * velMatchConst, diff[1] * velMatchConst];
}

function getAlignment(boid, localBoids, alignmentConst) {
    var avgHeading = getAvgHeading(localBoids);
    var diff = avgHeading - boid.direction;
    var newDirection = boid.direction + diff * alignmentConst;
    var velMag = magnitude(boid.vel);
    var newVel = convert(velMag, newDirection, [0, 0]);
    return [newVel[0] - boid.vel[0], newVel[1] - boid.vel[1]];
}

// function getFear(boid, localPredators, fearConst) {
//     var fearX = 0;
//     var fearY = 0;
//     await localPredators.forEach((pred) => {
//         var fearForce =
//     })
// }

function getAvoid(boid, localObs, proximity) {
    var avoidX = 0;
    var avoidY = 0;
    localObs.forEach((obs) => {
        var velMag = magnitude(boid.vel);
        console.log(
            "absolute angle towards obs: " +
                getAngle(boid.pos, obs.pos) / Math.PI +
                " * pi"
        );
        var a = Math.abs(boid.direction - getAngle(boid.pos, obs.pos));
        var x = velMag * Math.abs(Math.cos(a));
        console.log("angle relative to direction: " + a / Math.PI + " * pi");
        console.log("component magnitude: " + x);
        var reverse = convert(x, getAngle(boid.pos, obs.pos), [0, 0]);
        avoidX -= reverse[0];
        avoidY -= reverse[1];
    });
    console.log("result: " + [avoidX, avoidY]);
    return [avoidX, avoidY];
}

async function getAvoidSteer(boid, localObs, proximity, avoidSteerConst) {
    var avoidX = 0;
    var avoidY = 0;
    await localObs.forEach((obs) => {
        console.log("boid pos: " + boid.pos);
        console.log("boid direction: " + degrees(boid.direction));
        var r = obs.radius + proximity;
        var d = distance(boid.pos, obs.pos);
        var theta1 = Math.asin(r / d);
        console.log("theta1: " + degrees(theta1));
        if (isNaN(theta1)) return;
        var theta2 = getAngle(boid.pos, obs.pos) - boid.direction;
        if (Math.abs(theta2) % (0.5 * Math.PI) == 0) theta2 = 0;
        console.log("theta2: " + degrees(theta2));
        var angleToAvoid = theta1 - theta2;
        console.log("angleToAvoid: " + degrees(angleToAvoid));
        var tangentDist = Math.sqrt(Math.pow(d, 2) - Math.pow(r, 2));
        var addOption = convert(
            tangentDist,
            boid.direction + angleToAvoid,
            boid.pos
        );
        var subOption = convert(
            tangentDist,
            boid.direction - angleToAvoid,
            boid.pos
        );
        if (
            Math.abs(distance(addOption, obs.pos) - r) <
            Math.abs(distance(subOption, obs.pos) - r)
        )
            var targetDirection = boid.direction + angleToAvoid;
        else var targetDirection = boid.direction - angleToAvoid;
        console.log("targetDirection: " + degrees(targetDirection));
        var diff = targetDirection - boid.direction;
        var newDirection = boid.direction + diff * avoidSteerConst;
        var newVel = convert(magnitude(boid.vel), newDirection, [0, 0]);
        //return [newVel[0] - boid.vel[0], newVel[1] - boid.vel[1]];
        avoidX = avoidX + newVel[0] - boid.vel[0];
        avoidY = avoidY + newVel[1] - boid.vel[1];
    });
    return [avoidX, avoidY];
}

function getGoalSeek(boid, goalPos, goalSeekConst) {
    var angle = getAngle(boid.pos, goalPos);
    return convert(goalSeekConst, angle, [0, 0]);
}

function getAvgPosition(boids) {
    var x = 0;
    var y = 0;
    boids.forEach((b) => {
        x += b.pos[0];
        y += b.pos[1];
    });
    return [x / boids.length, y / boids.length];
}

function getAvgVelocity(boids) {
    var x = 0;
    var y = 0;
    boids.forEach((b) => {
        x += b.vel[0];
        y += b.vel[1];
    });
    return [x / boids.length, y / boids.length];
}

function getAvgHeading(boids) {
    var total = 0;
    boids.forEach((b) => {
        total += b.direction;
    });
    return total / boids.length;
}
function randomInt(min, max) {
    min = Math.ceil(min);
    max = Math.floor(max);
    return Math.floor(Math.random() * (max - min)) + min;
}

function randomDec(min, max) {
    return Math.random() * (max - min) + min;
}

function distance(pos1, pos2) {
    return Math.sqrt(
        Math.pow(pos2[0] - pos1[0], 2) + Math.pow(pos2[1] - pos1[1], 2)
    );
}

function degrees(rad) {
    return (rad * 180) / Math.PI;
}

function getAngle(pos1, pos2) {
    var x1 = pos1[0];
    var x2 = pos2[0];
    var y1 = pos1[1];
    var y2 = pos2[1];
    var a = Math.abs(Math.atan((y2 - y1) / (x2 - x1)));
    if (x2 >= x1 && y2 >= y1) return a;
    else if (x2 < x1 && y2 >= y1) return Math.PI - a;
    else if (x2 <= x1 && y2 < y1) return Math.PI + a;
    else if (x2 > x1 && y2 < y1) return 2 * Math.PI - a;
}

function getUnitVector(pos) {
    var magnitude = distance(pos, [0, 0]);
    return [pos[0] / magnitude, pos[1] / magnitude];
}

function magnitude(vector) {
    return Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2));
}

var game;
var state = {};
var running;
function startGame() {
    state = {};
    var boids = [];
    state.obstacles = [];
    state.predators = [];
    state.goalSeeking = {};
    state.goalSeeking.bool = false;
    state.goalSeeking.pos = [0, 0];
    //state.obstacles = [createObstacle([300, 300], 50, "#ffff00")];
    //state.predators = [createPredator([200, 200], [2, 0], "#ff0000")];
    //var obstacles = [];
    numOfBoids = 1000;
    var pos = [0, 0];
    var vel = [0, 0];
    var color = "#ffffff";
    for (i = 0; i < numOfBoids; i++) {
        pos = [randomInt(0, width), randomInt(0, height)];
        vel = [randomDec(-1, 1), randomDec(-1, 1)];
        // pos = [200, 200];
        // vel = [0, -2];
        boids.push(createBoid(pos, vel, color));
    }
    // var pos = [200, 0];
    // var vel = [0, -1];
    // var color = "#ffffff";
    // boids.push(createBoid(pos, vel, color));
    state.boids = boids;
    running = true;
    game = setInterval(async function () {
        state = await update(state);
        render(state);
    }, 1000 / frameRate);
}

function resetGame() {
    document.getElementById("resetButton").blur();
    clearInterval(game);
    startGame();
}

document.addEventListener("keypress", function (event) {
    if (event.keyCode == 32) handleFreeze();
    else if (event.keyCode == 115) handleGoalSeeking();
    else if (event.keyCode == 114) resetGame();
});

function handleFreeze() {
    document.getElementById("freezeButton").blur();
    if (running) {
        clearInterval(game);
        running = false;
        document.getElementById("freezeButton").innerHTML = "Unfreeze (space)";
    } else {
        running = true;
        game = setInterval(async function () {
            state = await update(state);
            render(state);
        }, 1000 / frameRate);
        document.getElementById("freezeButton").innerHTML = "Freeze (space)";
    }
}

function handleGoalSeeking() {
    document.getElementById("goalSeekButton").blur();
    if (state.goalSeeking.bool) {
        state.goalSeeking.bool = false;
        document.getElementById("goalSeekButton").innerHTML = "Seek mouse (s)";
    } else {
        state.goalSeeking.bool = true;
        document.getElementById("goalSeekButton").innerHTML =
            "Stop seeking mouse (s)";
    }
}

document.getElementById("world").addEventListener(
    "click",
    function (event) {
        var pos = [event.offsetX, event.offsetY];
        var vel = [randomDec(-1, 1), randomDec(-1, 1)];
        var color = "#ff8585";
        state.boids.push(createBoid(pos, vel, color));
    },
    false
);

document
    .getElementById("world")
    .addEventListener("mousemove", function (event) {
        state.goalSeeking.pos = [event.offsetX, event.offsetY];
    });

startGame();
// var testList = [
//     [1, 0],
//     [1, 0.5],
//     [1, 1],
//     [0.5, 1],
//     [0, 1],
//     [-0.5, 1],
//     [-1, 1],
//     [-1, 0.5],
//     [-1, 0],
//     [-1, -0.5],
//     [-1, -1],
//     [-0.5, -1],
//     [0, -1],
//     [0.5, -1],
//     [1, -1],
//     [1, -0.5],
// ];
// testList.forEach((pos) => console.log(degrees(getAngle([0, 0], pos))));
