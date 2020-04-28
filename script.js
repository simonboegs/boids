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
    var repelConst = document.getElementById("repelConstInput").value;
    var avoidConst = 1;
    var cohesionConst =
        document.getElementById("cohesionConstInput").value / 10;
    var velMatchConst =
        document.getElementById("velMatchConstInput").value / 10;
    var border = 100;
    //var minSpeed = document.getElementById("minSpeedInput");
    var minSpeed = 1;
    var maxSpeed = document.getElementById("maxSpeedInput");
    boids = state.boids;
    await boids.forEach(async (boid) => {
        var localBoids = await getLocalBoids(boid, boids, range, angleOfSight);
        var localObstacles = getLocalObstacles(boid, state.obstacles, range);
        if (localObstacles.length > 0) {
            var avoid = getAvoid(boid, localObstacles, avoidConst);
            boid.vel[0] += avoid[0];
            boid.vel[1] += avoid[1];
        }
        if (localBoids.length > 0) {
            var repel = await getRepel(boid, localBoids, repelConst);
            var cohesion = await getCohesion(boid, localBoids, cohesionConst);
            var velMatch = await velocityMatch(boid, localBoids, velMatchConst);
            boid.vel[0] += repel[0] + cohesion[0] + velMatch[0];
            boid.vel[1] += repel[1] + cohesion[1] + velMatch[1];
        }
        boid.direction = getDirection(boid.vel);
        var velMag = distance([0, 0], boid.vel);
        if (velMag < minSpeed)
            boid.vel = convert(minSpeed, boid.direction, [0, 0]);
        else if (velMag > maxSpeed)
            boid.vel = convert(maxSpeed, boid.direction, [0, 0]);
    });

    await boids.forEach((boid) => {
        boid.pos[0] += boid.vel[0];
        boid.pos[1] += boid.vel[1];

        if (boid.pos[0] < -border) boid.pos[0] += 2 * border + width;
        else if (boid.pos[0] > width + border)
            boid.pos[0] = boid.pos[0] - 2 * border - width;
        if (boid.pos[1] < -border) boid.pos[1] += 2 * border + height;
        else if (boid.pos[1] > height + border)
            boid.pos[1] = boid.pos[1] - 2 * border - height;
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

function createBoid(pos, vel, color) {
    const obj = {};
    obj.pos = pos;
    obj.vel = vel;
    obj.color = color;
    obj.width = 5;
    obj.height = 10;
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

function getLocalObstacles(boid, obstacles, range) {
    var localObs = [];
    obstacles.forEach((obs) => {
        var dist =
            Math.sqrt(
                Math.pow(boid.pos[0] - obs.pos[0], 2) +
                    Math.pow(boid.pos[1] - obs.pos[1], 2)
            ) - obs.radius;
        if (dist < 0) state.boids.splice(state.boids.indexOf(boid), 1);
        if (dist <= range) {
            localObs.push(obs);
            console.log(dist);
        }
    });
    return localObs;
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

function getAvoid(boid, localObs, avoidConst) {
    var avoidX = 0;
    var avoidY = 0;
    localObs.forEach((obs) => {
        var avoidForce =
            avoidConst / (distance(boid.pos, obs.pos) - obs.radius);
        var avoidDirection = getAngle(obs.pos, boid.pos);
        var avoid = convert(avoidForce, avoidDirection, [0, 0]);
        avoidX += avoid[0];
        avoidY += avoid[1];
    });
    return [avoidX, avoidY];
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

function getAngle(pos1, pos2) {
    var a = Math.atan(
        Math.abs(pos1[1] - pos2[1]) / Math.abs(pos1[0] - pos2[0])
    );
    if (pos2[0] - pos1[0] >= 0) {
        if (pos2[1] - pos1[1] >= 0) return a;
        else return a + Math.PI * 1.5;
    } else {
        if (pos2[1] - pos1[1] >= 0) return a + Math.PI / 2;
        else return a + Math.PI;
    }
}

function getUnitVector(pos) {
    var magnitude = distance(pos, [0, 0]);
    return [pos[0] / magnitude, pos[1] / magnitude];
}

var game;
var state = {};
var running;
function startGame() {
    state = {};
    var boids = [];
    var obstacles = [createObstacle([200, 200], 50, "#ffff00")];
    numOfBoids = 200;
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
    state.boids = boids;
    state.obstacles = obstacles;
    running = true;
    game = setInterval(async function () {
        state = await update(state);
        render(state);
    }, 1000 / frameRate);
}

function resetGame() {
    clearInterval(game);
    startGame();
}

function freeze() {
    if (running) {
        clearInterval(game);
        running = false;
    }
}

function unfreeze() {
    if (!running) {
        running = true;
        game = setInterval(async function () {
            state = await update(state);
            render(state);
        }, 1000 / frameRate);
    }
}

document.getElementById("world").addEventListener(
    "click",
    function (event) {
        var pos = [event.pageX, event.pageY];
        var vel = [randomDec(-1, 1), randomDec(-1, 1)];
        var color = "#ff8585";
        state.boids.push(createBoid(pos, vel, color));
    },
    false
);

startGame();
