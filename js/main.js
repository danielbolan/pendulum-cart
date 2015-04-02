(function(){
  var sim = new Simulation({
    cart: { l: 3 }, // yardstick is easier to balance than a pencil
    canvas: { parentElem: document.getElementById('container') }
  });

  var movingLeft = false;
  var movingRight = false;

  window.onkeydown = function( e ) {
    console.log( e.keyIdentifier + ' down');
    movingRight = (e.which == 39);
    movingLeft = (e.which == 37);
  };

  window.onkeyup = function( e ) {
    console.log( e.keyIdentifier + ' up');
    if (e.which == 39) movingRight = false;
    if (e.which == 37) movingLeft = false;
  };

  var planner = {
    getNextControl: function( state ) {
      var u = 0;
      if (movingRight) u += 2;
      if (movingLeft) u -= 2;
      return u;
    }
  };

  sim.setPlanner( planner );
  sim.run();
})();
