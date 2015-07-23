var myapp = (function(){
    var puttData = [];
    var width = 600;
    var height = 400;
    var animated = false;


    function setCanvasDim() {
        width = window.innerWidth;
        height = window.innerHeight;
        $("#puttCanvas").attr("width", width);
        $("#puttCanvas").attr("height", height / 2.0);
        console.log("width: "+$("#puttCanvas").attr("width"));
        console.log("set canvas dimensions");
        putterDemo("puttCanvas");
    };

    function getPuttData() {
        setCanvasDim();
        
        console.log("getting put data");
        var putt = Parse.Object.extend("Putt");
        var query = new Parse.Query(putt);
        query.get("12fz4AHTDK", {
            success: function(data) {
                puttData = data.get("frames");   
                console.log("got data");
                console.log(puttData);   
            },
            error: function(object, error) {
                console.log("Error getting putt data");
                console.log(error);
            }
        });
    };

    function animatePutt() {
        if(animated) {
            return;
        }else {
            window.requestAnimFrame = (function(callback) {
                return window.requestAnimationFrame || window.webkitRequestAnimationFrame || window.mozRequestAnimationFrame || window.oRequestAnimationFrame || window.msRequestAnimationFrame ||
                function(callback) {
                  window.setTimeout(callback, 1000 / 60);
                };
            })();
        }

      function drawRectangle(myRectangle, context) {
        context.beginPath();
        context.rect(myRectangle.x, myRectangle.y, myRectangle.width, myRectangle.height);
        context.fillStyle = '#8ED6FF';
        context.fill();
        context.lineWidth = myRectangle.borderWidth;
        context.strokeStyle = 'black';
        context.stroke();
      }
      function animate(myRectangle, canvas, context, frame, scale) {
        // update
        //var time = (new Date()).getTime() - startTime;

        //var linearSpeed = 100;
        // pixels / second
        //var newX = linearSpeed * time / 1000;
        console.log("frame: "+frame);
        var newX = puttData[frame][0] / scale;
        var newY = puttData[frame][1] / scale;
        var newZ = puttData[frame][2] / scale;

        /*if(newX < canvas.width - myRectangle.width - myRectangle.borderWidth / 2) {
          myRectangle.x = newX;
        }*/
        
        myRectangle.x = myRectangle.x + newX;
       // myRectangle.y = myRectangle.y + newY;

        // clear
        context.clearRect(0, 0, canvas.width, canvas.height);

        drawRectangle(myRectangle, context);

        // request new frame
        requestAnimFrame(function() {
          if(frame+1 < puttData.length) {
            animate(myRectangle, canvas, context, frame+1, scale);
          }else {
            animated = false;
            console.log("animation completed");
            return;
          }
        });
      }
      var canvas = document.getElementById('puttCanvas');
      var context = canvas.getContext('2d');

      var myRectangle = {
        x: (width / 2.0) - 25,
        y: (height / 4.0) - 50,
        width: 50,
        height: 100,
        borderWidth: 5
      };

      drawRectangle(myRectangle, context);

      // wait one second before starting animation
      setTimeout(function() {    
        console.log(puttData);
        if(puttData != [] && puttData != null) {   
            animated = true;    
            animate(myRectangle, canvas, context, 0, 10.0);
        }else {
            console.log("no putt data loaded");
        }
      }, 1000);
    };


function buildCube(g, width, colors) // pass width and array of 6 colors
{
  var sq = ['M',0,0,0, 'L',width,0,0, width,width,0, 0,width,0, 'z'],
      foldTbl = [-90, 90, -90, 90, -90, 90],
      bend = -90,
      moveTbl_1 = [-width, 0, -width, 0, -width, 0],
      moveTbl_2 = [width, 0, width, 0, width, 0],
      faces = g.createGroup3D(),
      side,
      i;

  for (i=0; i<6; i++)
  {
    side = g.compileShape3D(sq, colors[i]);
    side.backHidden = true;
    faces.addObj(side);
    faces.translate(0, moveTbl_1[i], 0);
    faces.rotate(0, 0, 1, foldTbl[i]);
    faces.rotate(0, 1, 0, bend);
    faces.translate(0, moveTbl_2[i], 0);
  }
  return faces;
}

function buildPutter(g, width, colors) //pass width and array of 6 colors
{
    var sq = ['M',0,0,0, 'L',width,0,0, width,width,0, 0,width,0, 'z'],
      foldTbl = [-90, 90, -90, 90, -90, 90],
      bend = -90,
      moveTbl_1 = [-width, 0, -width, 0, -width, 0],
      moveTbl_2 = [width, 0, width, 0, width, 0],
      faces = g.createGroup3D(),
      side,
      i;

  for (i=0; i<6; i++)
  {
    side = g.compileShape3D(sq, colors[i]);
    side.backHidden = true;
    faces.addObj(side);
    faces.translate(0, moveTbl_1[i], 0);
    faces.rotate(0, 0, 1, foldTbl[i]);
    faces.rotate(0, 1, 0, bend);
    faces.translate(0, moveTbl_2[i], 0);
  }
  return faces;
}




function putterDemo(scrnID)
{
  var g = new Cango3D(scrnID),
      newPos = {x:0, y:0, z:0},  // avoid creating Objects in event handlers
      taggedFace, cube1,
      width = 20,
      colors1 = ["green", "green", "green", "green", "green", "green"],
      bottom,
      putterhead;

      console.log(g.cnvs.offsetWidth);







    function makePutterHeadlayer(input, z) {
        
        input = g.createGroup3D();
        
        plateL = g.compileShape3D(shapes3D.circle(20), "#D3D3D3");
        plateL.transform.translate(0,50,z);
        input.addObj(plateL);
    
        plateR = g.compileShape3D(shapes3D.circle(20), "#D3D3D3");
        plateR.transform.translate(0,-50,z);
        input.addObj(plateR);

        headAppendL = g.compileShape3D(shapes3D.square(20), "#D3D3D3");
        headAppendL.transform.translate(0,40,z);
        input.addObj(headAppendL);

        headAppendLhalf = g.compileShape3D(shapes3D.square(20), "#D3D3D3");
        headAppendLhalf.transform.translate(0,20,z);
        input.addObj(headAppendLhalf);

        headAppendR = g.compileShape3D(shapes3D.square(20), "#D3D3D3");
        headAppendR.transform.translate(0,0,z);
        input.addObj(headAppendR);

        headAppendRhalf = g.compileShape3D(shapes3D.square(20), "#D3D3D3");
        headAppendRhalf.transform.translate(0,-20,z);
        input.addObj(headAppendRhalf);

        headAppendC =g.compileShape3D(shapes3D.square(20), "#D3D3D3");
        headAppendC.transform.translate(0,-40,z);
        input.addObj(headAppendC);
        
        return input;
    } 

  function movePutter()
  {
    // use target's parent group drawing origin as reference
    bottom.transform.rotate(0,1,0,-25);
    g.renderFrame(bottom);
   /* newPos.x = mousePos.x-this.grabOfs.x;
    newPos.y = mousePos.y-this.grabOfs.y;
    newPos.z = mousePos.z-this.grabOfs.z;
    cube1.transform.reset();          // reset to identity matrix
    cube1.transform.translate(newPos.x, newPos.y, newPos.z);

    g.renderFrame(grp);*/
  }

  g.setWorldCoords3D(-100, -80, 300);
  g.setFOV(45);
  g.setPropertyDefault("backgroundColor", "lightyellow");



   bottom = g.createGroup3D();

   putterhead = makePutterHeadlayer(bottom,0);
   bottom.addObj(putterhead);

    var layers = 10;
    var i = 1;
    for (i; i < layers; i++){
        bottom.addObj(makePutterHeadlayer(bottom,i))
    }

    bottom.backHidden = true;

    bottom.transform.translate(35,-60,-10);
    bottom.transform.rotate(1,1,0,30);
    
    g.render(bottom);




  /*cube1 = buildCube(g, width, colors1);
  cube1.rotate(1, 1, 0, 30);
  cube1.translate(35, -60, -10);
  */
  setInterval(movePutter, 500);
 // g.render(grp);
}

    function drawDemo(cvsID)
{
  var stick, plate, plateNstick,
      g = new Cango3D(cvsID);  // create a graphics context

  function turnPlate()
  {
    plateNstick.transform.rotate(0, 1, 0, -25);     // apply 25 deg rotation to matrix
    g.renderFrame(plateNstick);           // request a re-draw
  }

  g.setPropertyDefault("backgroundColor", "aliceblue");
  g.setWorldCoords3D(-75, -120, 150);
  g.setLightSource(0, 100, 200);

  stick = g.compilePath3D(["M",0,0,0, "Q", 0,50,0, -15, 100, 0], "sienna", 3);
  stick.rotate(0, 1, 0, 90);  // rotate out of XY plane
  plate = g.compileShape3D(shapes3D.circle(50), "yellow", "yellow");
  plate.rotate(1, 0, 0, 75);  // flip to near horizontal
  stick.translate(0, -96, 0); // move down Y axis under plate
  // make a group comprising stick and plate
  plateNstick = g.createGroup3D(stick, plate);

  setInterval(turnPlate, 50)        // keep doing this forever
}
   




    return {
        init: function() {
            console.log("starting up");
            Parse.initialize("iAFEw9XwderX692l0DGIwoDDHcLTGMGtcBFbgMqb", "d4q6T1YxleQ2za17PdTtfaAD8x6tAB9rdW4y9vMD");
            $(window).load(getPuttData);
            jQuery("#startButton").click(animatePutt);
            //$(window).load(putterDemo("puttCanvas"))
            //console.log("Client-side app starting up")
            
			//jQuery("#testbutton").click(test);
            
        }
    }
})();
jQuery(myapp.init);



