var myapp = (function(){
    var accelData = [];
    var gyroData = [];
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
        if($("#puttCanvas").length) {
            console.log("found canvas");
            setCanvasDim();
            
            console.log("getting put data");
            var putt = Parse.Object.extend("Putt");
            var query = new Parse.Query(putt);
            query.get("12fz4AHTDK", {
                success: function(data) {
                    accelData = data.get("frames");   
                    gyroData = data.get("gyro");
                    console.log("got data");
                    console.log(accelData);   
                },
                error: function(object, error) {
                    console.log("Error getting putt data");
                    console.log(error);

                }

            });
        }
        
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
      faces = g.createGroup3D(),
      side,
      translateX = [-width/2,-width/2,-width/2,-width/2,-width/2,-width/2],
      translateY = [-width/2,-width/2,-width/2, -width/2,-width/2,-width/2],
      translateZ = [width/2,width/2,width/2,width/2,width/2,width/2],
      rotateX = [1,1,1,1,0,0],
      rotateY = [1,0,0,0,1,1],
      rotateZ = [1,0,0,0,0,0],
      rotateMag = [0,90,-90,180,90,-90],
      i;

  for (i=0; i<6; i++)
  {
    console.log(i)
    side = g.compileShape3D(sq, colors[i]);
    side.translate(translateX[i], translateY[i], translateZ[i]);
    side.rotate(rotateX[i], rotateY[i], rotateZ[i], rotateMag[i]);
    side.backHidden = true;
    faces.addObj(side);
  }
    // Axes added for visual debugging
    centerX = g.compilePath3D(["M",-50,0,0, "L",50,0,0], "black", 1);
    faces.addObj(centerX);
    centerY = g.compilePath3D(["M",0,-50,0, "L",0,50,0], "black", 1);
    faces.addObj(centerY);
    centerZ = g.compilePath3D(["M",0,0,-50, "L",0,0,50], "black", 1);
    faces.addObj(centerZ);
        
  return faces;
}




function putterDemo(scrnID)
{
  var g = new Cango3D(scrnID),
      newPos = {x:0, y:0, z:0},  // avoid creating Objects in event handlers
      taggedFace, cube1,
      width = 20,
      colors1 = ["red","orange","yellow","green","blue","purple"],
      iter = 0,
      coordsX = -150,
      coordsY = -80,
      coordsZ = 300,
      incCoords = 0;

      console.log(g.cnvs.offsetWidth);



  function movePutter()
  {
    // use target's parent group drawing origin as reference
    //cube1.transform.rotate(0,1,0,-25);
    /*cube1.transform.rotate(1,0,0,gyroData[iter][0]);
    cube1.transform.rotate(0,1,0,gyroData[iter][1]);
    cube1.transform.rotate(0,0,1,gyroData[iter][2]);*/
    g.setWorldCoords3D(coordsX+incCoords, coordsY+incCoords, coordsZ);
    incCoords = incCoords + 1;
      
    cube1.rotate(1,1,1,10);
    g.renderFrame(cube1);
   /* newPos.x = mousePos.x-this.grabOfs.x;
    newPos.y = mousePos.y-this.grabOfs.y;
    newPos.z = mousePos.z-this.grabOfs.z;
    cube1.transform.reset();          // reset to identity matrix
    cube1.transform.translate(newPos.x, newPos.y, newPos.z);

    g.renderFrame(grp);*/
  }

  g.setWorldCoords3D(coordsX, coordsY, coordsZ);
  g.setFOV(45);
  g.setPropertyDefault("backgroundColor", "lightyellow");




    //bottom.transform.translate(35,-60,-10);
    //bottom.transform.rotate(1,1,0,30);
    
    //g.render(bottom);




  cube1 = buildCube(g, width, colors1);


  //cube1.rotate(1, 1, 0, 30);
  //cube1.translate(35, -60, -10);
  //g.render(cube1);
//cube1.transform.rotate(0,1,0,30);
  setInterval(movePutter, 100);
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



