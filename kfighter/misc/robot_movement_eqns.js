var f = createFont("monospace");
textFont(f);

var s = 50;
var c = [200, 200];
var t = 0;
var t_step = 1/60;
var v;
var lineno = 0;

function cp(a, b) {
    return Object.assign({}, a, b);
}

function computeForward(vin) {
    var v = cp(vin);
    v.p = v.S0*v.t*v.t*v.t*v.t/24 + v.J0*v.t*v.t*v.t/6 +
        v.a0*v.t*v.t/2 + v.v0*v.t +v.p0;
    v.v = v.S0*v.t*v.t*v.t/6 + v.J0*v.t*v.t/2 + v.a0*v.t + v.v0;
    v.a = v.S0*v.t*v.t/2 + v.J0*v.t + v.a0;
    v.J = v.S0*v.t + v.J0;
    v.S = v.S0;
    return v;
}

function computeEnd(vin) {
    var vtmp = computeForward(cp(vin, {t: vin.t1}));
    vin.p1 = vtmp.p;
    vin.v1 = vtmp.v;
    vin.a1 = vtmp.a;
    vin.J1 = vtmp.J;
    vin.S1 = vtmp.S;
}

function computeVarsFromS0(v) {
    v.J0 = -v.S0*v.t1/2 + 6*(2*v.p0 - 2*v.p1 + v.t1*v.v0 + v.t1*v.v1)/v.t1/v.t1/v.t1;
    v.a0 = v.S0*v.t1*v.t1/12 - 2*(3*v.p0 - 3*v.p1 + 2*v.t1*v.v0 + v.t1*v.v1)/v.t1/v.t1; 
    computeEnd(v);
}

function minAbsIntersection(x1, r1, x2, r2) {
    // |x1 + r1*x| = |x2 + r2*x|
    if (r1 === r2 &&  r1 === -r2) {return 0;}
    else if (r1 === r2) {return (x2 + x1) / (r2 + r1);}
    else if (r1 === -r2) {return (x2 - x1) / (r2 - r1);}
    else {
        var a = (x2 - x1) / (r2 - r1);
        var b = (x2 + x1) / (r2 + r1);
        if (Math.abs(x1 + r1*a) < Math.abs(x1 + r1*b)) {
            return a;
        } else {
            return b;
        }
    }
}

function computeS0ForMinA(v) {
    // a1 = v.S0*v.t1*v.t1/2 + (J0_1*v.S0 + J0_0)*v.t1 + (a0_1*v.S0 + a0_0);
    var a0_0 = -2*(3*v.p0 - 3*v.p1 + 2*v.t1*v.v0 + v.t1*v.v1)/v.t1/v.t1; 
    var a0_1 = v.t1*v.t1/12;
    var J0_0 = 6*(2*v.p0 - 2*v.p1 + v.t1*v.v0 + v.t1*v.v1)/v.t1/v.t1/v.t1;
    var J0_1 = -v.t1/2;
    var a1_0 = J0_0*v.t1 + a0_0;
    var a1_1 = v.t1*v.t1/2 + J0_1*v.t1 + a0_1;
    //TODO: Handle case where the global min is the quadratic min.
    return minAbsIntersection(a0_0, a0_1, a1_0, a1_1);
}

function computeInitialVals(v) {
    //for (var x in v){text(x+":"+v[x],300,12*lineno++);}
    v.S0 = computeS0ForMinA(v);
    computeVarsFromS0(v);
    //lineno++;
    //for (var x in v){text(x+":"+v[x],300,12*lineno++);}
}

function pad(x, c) {
    while(x.length<c){x="0"+x;}
    return x;
}

function valtostr(x) {
    if (x instanceof Array) {
        return valtostr(x[0]);
    }
    var s = "";
    if (x < 0) {s += '-'; x = -x;}
    else {s+='+';}
    var rd = round(x*100);
    s += pad(""+~~(rd/100),2);
    s += "."+ pad(""+rd%100,2);
    return s;
}

function writeln(name, val, min, max) {
    lineno++;
    var str = name+"="+valtostr(val);
    if (min !== undefined) {str+=" ["+valtostr(min)+", "+valtostr(max)+"]";} 
    text(str, 2, 12*lineno);
}

function writeVars(v) {
    lineno=0;
    writeln('t', v.t, 0, v.t1);
    writeln('p', v.p, v.p0, v.p1);
    writeln('v', v.v, v.v0, v.v1);
    writeln('a', v.a, v.a0, v.a1);
    writeln('J', v.J, v.J0, v.J1);
    writeln('S', v.S, v.S0, v.S1);
}

smooth();
frameRate(30);


    
function draw() {
    background(255);
    fill(0,0,0);
    v = {
        t1: 3,
        p0: -2, p1: 3,
        v0: 0, v1: -4,
        t: t,
    };
    
    computeInitialVals(v);
    v = computeForward(v);
    writeVars(v);
    ellipse(v.p*s + c[0], c[1], 10, 10);
    
    t += t_step;
    if (t >= v.t1) {t = 0;}
}
