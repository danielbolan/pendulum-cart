var State = function( copy ) {
  if (copy) {
    this.x = copy.x;
    this.v = copy.v;
    this.theta = copy.theta;
    this.omega = copy.omega;
    this.t = copy.t;
  } else {
    this.x = 0;
    this.v = 0;
    this.theta = 0;
    this.omega = 0;
    this.t = 0;
  }
};

var Simulation = function( params ) {
  var self = this;

  params = params || {};
  params.cart = params.cart || { M: 0.6, m: 0.1, l: 3 };
  params.cart.M = params.cart.M || 0.6;
  params.cart.m = params.cart.m || 0.1;
  params.cart.l = params.cart.l || 3;
  params.canvas = params.canvas || { parentElem: document.body, width: 600, height: 400 };
  params.canvas.parentElem = params.canvas.parentElem || document.body;
  params.canvas.width = params.canvas.width || 600;
  params.canvas.height = params.canvas.height || 400;

  var planner = {
    initialize: function() {},
    getNextControl: function() { return 0; }
  };

  var drawState = (function() {
    var canvas = document.createElement('canvas');
    canvas.width = params.canvas.width;
    canvas.height = params.canvas.height;
    canvas.style.width = canvas.width + 'px';
    canvas.style.height = canvas.height+ 'px';;
    params.canvas.parentElem.appendChild(canvas);
    var ctx = canvas.getContext('2d');

    // visual parameters, don't effect simulation
    var tickSeparation = 1; //  distance between tickmarks on track, in meters
    var cartDims = {width: 0.5, height: 0.25}; // in meters
    var pendulumRadius = 0.1; // in meters

    //return the function that will actually get called
    //(the rest of this function is done so that we don't pollute the namespace)
    return function showState( state ){
      ctx.clearRect( 0, 0, canvas.width, canvas.height );
      var metersPerScreen = (canvas.width/canvas.height) * params.cart.l * 2 + 1;
      var metersPerPixel = canvas.width / metersPerScreen;

      // track and tickmarks
      ctx.lineWidth = 1;
      ctx.strokeStyle = '#aaa';
      ctx.beginPath();
      ctx.moveTo( 0, canvas.height / 2 );
      ctx.lineTo( canvas.width, canvas.height / 2 )
      for( var i=-metersPerScreen; i<=metersPerScreen+1; i+=tickSeparation ) {
        var x = Math.floor(state.x) - state.x;
        x = canvas.width / 2 + (x+i)*metersPerPixel;
        ctx.moveTo( x, canvas.height / 2 + 0.05 * metersPerPixel );
        ctx.lineTo( x, canvas.height / 2 - 0.05 * metersPerPixel );
      }
      ctx.stroke();

      // cart
      ctx.fillStyle = '#aff';
      ctx.fillRect(
        canvas.width/2 - cartDims.width*metersPerPixel/2,
        canvas.height/2 - cartDims.height*metersPerPixel/2,
        cartDims.width * metersPerPixel,
        cartDims.height * metersPerPixel
      )
      ctx.strokeStyle = '#699';
      ctx.strokeRect(
        canvas.width/2 - cartDims.width*metersPerPixel/2,
        canvas.height/2 - cartDims.height*metersPerPixel/2,
        cartDims.width * metersPerPixel,
        cartDims.height * metersPerPixel
      )

      // pendulum
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#966';
      ctx.fillStyle = '#faa';
      ctx.beginPath();
      var tipX = canvas.width/2 + Math.cos(state.theta-Math.PI/2) * params.cart.l * metersPerPixel;
      var tipY = canvas.height/2 - Math.sin(state.theta-Math.PI/2) * params.cart.l * metersPerPixel;
      ctx.moveTo( canvas.width / 2, canvas.height / 2 );
      ctx.lineTo( tipX, tipY )
      ctx.stroke();
      ctx.beginPath();
      ctx.arc( tipX, tipY, pendulumRadius * metersPerPixel, 0, 2*Math.PI, false );
      ctx.fill();
      ctx.stroke();
    };
  })();

  var kinematicDerivative = function kinematicDerivative( s, u ) {
    var M = params.cart.M;
    var m = params.cart.m;
    var l = params.cart.l;
    var g = 9.81;

    var sinth = Math.sin(s.theta);
    var costh = Math.cos(s.theta);

    var ds = new State();

    ds.x = s.v;
    ds.theta = s.omega;

    ds.v = m*g*sinth*costh;
    ds.v -= m*l*s.omega*s.omega*sinth;
    ds.v -= u;
    ds.v /= (m*costh*costh)-(M+m);

    ds.omega = m*l*s.omega*s.omega*sinth*costh;
    ds.omega += u*costh;
    ds.omega += (M+m)*g*sinth;
    ds.omega /= (m*l*costh*costh)-(M+m)*l;

    return ds;
  }

  var kinematicIntegral = function kinematicIntegral( s, ds, dt ) {
    var s_ = new State( s );
    s_.x += ds.x * dt;
    s_.v += ds.v * dt;
    s_.theta += ds.theta * dt;
    s_.omega += ds.omega * dt;
    return s_;
  }

  this.propagate =function rk4( state, control, dt ) {
    if (!dt) dt = 0.025;

    var ds1 = kinematicDerivative( state, control );
    var s1 = kinematicIntegral( state, ds1, dt/2 );
    var ds2 = kinematicDerivative( s1, control );

    var s2 = kinematicIntegral( state, ds2, dt/2 );
    var ds3 = kinematicDerivative( s2, control );

    var s3 = kinematicIntegral( state, ds3, dt );
    var ds4 = kinematicDerivative( s3, control );

    state = kinematicIntegral( state, ds1, dt/6 );
    state = kinematicIntegral( state, ds2, dt/3 );
    state = kinematicIntegral( state, ds3, dt/3 );
    state = kinematicIntegral( state, ds4, dt/6 );
    state.t += dt;

    return state;
  };

  this.setPlanner = function setPlanner( p ) {
    planner.initialize = p.initialize || planner.initialize;
    planner.getNextControl = p.getNextControl || planner.getNextControl;
  };

  this.run = function run() {
    var state = new State();
    planner.initialize( new State(state) );
    setInterval( function() {
      var control = planner.getNextControl( new State(state) ) || 0;
      state = self.propagate( state, control, 0.025 );
      drawState(state);
    }, 25);
  };
};
