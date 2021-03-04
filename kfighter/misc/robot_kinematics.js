function sin(v) {return Math.sin(v);}
function cos(v) {return Math.cos(v);}
var pi = Math.PI;
var f = createFont("monospace");
textFont(f);
var lineno = 1;
function assert(x) {
    if (!x) {
        undefined();
    }
}
function matrix(M, w, h) {
    M.w = w;
    M.h = h;
    M.dim = [w,h];
    assert(M.length === w*h);
    return M;
}
function vec(V) {return matrix(V, 1, V.length);}
function I(n) {
    var M = [];
    for (var i = 0; i < n; i++) {
        for (var j = 0; j < n; j++) {
            M.push(i === j ? 1 : 0);
        }
    }
    return matrix(M, n, n);
}
function transpose(M1) {
    var M2 = [];
    for (var i = 0; i < M1.w; i++) {
        for (var j = 0; j < M1.h; j++) {
            M2.push(M1[j*M1.w + i]);
        }
    }
    return matrix(M2,M1.h,M1.w);
}
function valtostr(v) {
    if (v instanceof Array && v.w && v.h) {
        if (v.w === 1) {return valtostr(transpose(v))+'T';}
        var a = [];
        var colwidths = [];
        for (var i = 0; i < v.h; i++) {
            var row = [];
            for (var j = 0; j < v.w; j++) {
                var x = valtostr(v[i*v.w+j]);
                if (!colwidths[j] || colwidths[j] < x.length) {
                    colwidths[j] = x.length;
                }
                row.push(x);
            }
            a.push(row);
        }
        var s = '[';
        if (v.h > 1){s+='\n';}
        for (var i = 0; i < v.h; i++) {
            for (var j = 0; j < v.w; j++) {
                var x = a[i][j];
                for (var k = 0; k < colwidths[j] + 1 - x.length; k++) {
                    s += ' ';
                }
                s += x;
            }
            s += '\n';
        }
        s = s.substring(0, s.length - 1);
        s += ']';
        return s;
    } else {
        return ''+(~~(v*10+0.5)/10);
    }
}
function writeln(name, val) {
    var s = valtostr(val);
    var increment = (s.split('\n')).length;
    text(name+'='+s, 2, 14*lineno);
    lineno += increment;
}
function mul() {
    var B = arguments[arguments.length-1];
    for (var a = arguments.length-2; a >= 0; a--) {
        var A = arguments[a];
        assert(A.w === B.h);
        var C = [];
        for (var j = 0; j < A.h; j++) {
            for (var i = 0; i < B.w; i++) {
                var v = 0;
                for (var k = 0; k < A.w; k++) {
                    v += A[j*A.w+k]*B[k*B.w+i];
                }
                C.push(v);
            }
        }
        B = matrix(C, B.w, A.h);
    }
    return B;
}
function rot_inv2(M) {return transpose(M,2,2);}
function mat_map(f, M) {
    var X = [];
    for (var i = 0; i < M.length; i++) {
        X.push(f(M[i]));
    }
    return matrix(X, M.w, M.h);
}
function scale(x) {return function(y) {return x*y;};}
function skew2(a) {return matrix([0,-a[0],a[0],0],2,2);}
function get_theta2(M) {return M[2];}
function exp2(M1) {
    var t = get_theta2(M1), c = cos(t), s = sin(t);
    return matrix([c,-s,s,c],2,2);
}
function log2(M1) {return skew2(vec([atan2(M1[2],M1[0])]));}
function trans2(R,p) {
    return matrix([R[0],R[1],p[0],R[2],R[3],p[1],0,0,1],3,3);}
function get_rot2(T) {return matrix([T[0],T[1],T[3],T[4]],2,2);}
function get_pos2(T) {return matrix([T[2],T[5]],1,2);}
function trans_inv2(T) {
    var R = transpose(get_rot2(T));
    var p = mul(R, mat_map(scale(-1), get_pos2(T)));
    return trans2(R, p);
}
var proj = matrix([20,0,200,0,-20,200,0,0,1],3,3);
function draw_arrow(x1,y1,x2,y2) {
    line(x1,y1,x2,y2);
    var theta = Math.atan2(y2-y1, x2-x1);
    var l = 6;
    line(x2, y2, x2 + l*cos(theta+3*pi/4), y2 + l*sin(theta+3*pi/4));
    line(x2, y2, x2 + l*cos(theta-3*pi/4), y2 + l*sin(theta-3*pi/4));
}
function draw_label(str, x,y,x1,y1) {
    var theta = Math.atan2(y1-y, x-x1);
    var offset = [4*cos(theta), -2*sin(theta)];
    if (theta < -5*pi/6) {
        textAlign(RIGHT, CENTER);
    } else if (theta < -4*pi/6) {
        textAlign(RIGHT, TOP);
    } else if (theta < -2*pi/6) {
        textAlign(CENTER, TOP);
    } else if (theta < -1*pi/6) {
        textAlign(LEFT, TOP);
    } else if (theta < 1*pi/6) {
        textAlign(LEFT, CENTER);
    } else if (theta < 2*pi/6) {
        textAlign(LEFT, BOTTOM);
    } else if (theta < 4*pi/6) {
        textAlign(CENTER, BOTTOM);
    } else if (theta < 5*pi/6) {
        textAlign(RIGHT, BOTTOM);
    } else {
        textAlign(RIGHT, CENTER);
    }
    text(str,x+offset[0],y+offset[1]);
    textAlign(LEFT, BASELINE);
}
function draw_axes(m,name) {
    var zero = mul(proj, m, vec([0,0,1]));
    var a = {};
    a.x = mul(proj, m, vec([1,0,1]));
    a.y = mul(proj, m, vec([0,1,1]));
    var xy = mul(proj, m, vec([1,1,1]));
    for (var n in a) {
        draw_arrow(zero[0],zero[1],a[n][0],a[n][1]);
        draw_label(n,a[n][0],a[n][1],zero[0],zero[1]);
    }
    draw_label('{'+name+'}',zero[0],zero[1],xy[0],xy[1]);
}

function Joint(a, ra, b, rb) {
    this.a = [a, b];
    this.r = [vec([ra[0],ra[1],1]), vec([rb[0],rb[1],1])];
    a.joints.push([this, this.r[0], 0]);
    b.joints.push([this, this.r[1], 1]);
}

function Segment(length, width, fixed) {
    this.length = length;
    this.width = width;
    this.fixed = fixed;
    this.joints = [];
    this.T = I(3);
    this.draw = function() {
        var hl = this.length/2, hw = this.width/2;
        var points = transpose(matrix([-hl,-hw,1,-hl,hw,1,hl,hw,1,hl,-hw,1],3,4));
        points = mul(proj, this.T, points);
        for (var i = 0; i < 4; i++) {
            var j = (i+1)%4;
            line(points[i],points[i+4],points[j],points[j+4]);
        }
        writeln('T', this.T);
        writeln('r0', this.r0);
        writeln('r1', this.r1);
    };
    this.compute_transform = function() {
        if (this.fixed) {
            this.computed = true;
            return;
        }
        for (var i = 0; i < this.joints.length; i++) {
            var joint = this.joints[i][0];
            var r = this.joints[i][1];
            var kind = this.joints[i][2];
            if (kind === 1) {
                var ix = ~~!kind;
                assert(joint.a[ix].computed);
                var R = exp2(skew2([joint.theta]));
                this.r0 = r;
                this.r1 = joint.r[ix];
                this.T = mul(
                    joint.a[ix].T,
                    trans2(R, joint.r[ix]),
                    trans2(I(2), mat_map(scale(-1),r)));
                this.computed = true;
            }
        }
    };
}
function Robot(joints, segments) {
    this.joints = joints;
    this.segments = segments;
    this.draw = function() {
        for (var i = 0; i < this.segments.length; i++) {
            this.segments[i].draw();
        }
    };
    this.set_angles = function(theta) {
        for (var i = 0; i < this.segments.length; i++) {
            this.segments[i].computed = false;
        }
        for (var i = 0; i < this.joints.length; i++) {
            this.joints[i].theta = theta[i];
        }
        for (var i = 0; i < this.segments.length; i++) {
            this.segments[i].compute_transform();
        }
    };
}
function make_open_chain_robot(segment_count, segment_length, base_T) {
    var hw = 0.1;
    var hl = segment_length/2;
    var segments = [new Segment(1.0, 1.0, true)];
    segments[0].T = base_T;
    for (var i = 0; i < segment_count; i++) {
        segments.push(new Segment(2*hl + 2*hw, 2*hw, false));
    }
    var joints = [new Joint(segments[0], [0,0], segments[1], [-hl,0])];
    for (var i = 1; i < segment_count; i++) {
        joints.push(new Joint(segments[i], [hl,0], segments[i+1], [-hl,0]));
    }
    return new Robot(joints, segments);
}

function draw() {
    lineno = 1;
    background(255, 255, 255);
    stroke(0, 0, 0);
    fill(153, 153, 153);
    var p = vec([0,0,1]);
    var sp = mul(proj, p);
    var Ts = trans2(I(2),vec([-1,-3]));
    draw_axes(Ts,'s');
    var robot = make_open_chain_robot(3, 2.0, Ts);
    robot.set_angles([0.9,-0.6,-1.0]);
    robot.draw();
}
