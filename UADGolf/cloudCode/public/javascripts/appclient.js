var myapp = (function(){
    var displacementData = [];
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
                    displacementData = data.get("frames");   
                    gyroData = data.get("gyro");
                    console.log("got data");
                    console.log(displacementData);   
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






  function buildPutter(g, width, colors) {
    
    // build the putter head
    var sq = [['M',0,0,0, 'L',width*4,0,0, width*4,width,0, 0,width,0, 'z']];
      sq.push(sq[0]);
      sq.push(sq[0]);
      sq.push(sq[0]);
      sq.push(['M',0,0,0, 'L',width,0,0, width,width,0, 0,width,0, 'z']);
      sq.push(sq[4]);
      sq.push(['M',0,0,0, 'L',width/2,0,0, width/2,width/2,0, 0,width/2,0, 'z']); // dot on putter head
    
      var faces = g.createGroup3D(),
      side,
      translateX = [-width*2,-width*2,-width*2,-width*2,-width/2,-width/2,-width/4],
      translateY = [0,-width/2,-width/2, -width,0,0,width/4],
      translateZ = [width/2,0,width,width/2,width*2,width*2,width/2+1],
      rotateX = [1,1,1,1,0,0,0],
      rotateY = [1,0,0,0,1,1,1],
      rotateZ = [1,0,0,0,0,0,0],
      rotateMag = [0,90,-90,180,90,-90,180],
      i;
    
    // make the 6 components of the putter head
    for (i=0; i<7; i++){
        //console.log(i)
        side = g.compileShape3D(sq[i], colors[i]);
        side.translate(translateX[i], translateY[i], translateZ[i]);
        side.rotate(rotateX[i], rotateY[i], rotateZ[i], rotateMag[i]);
        //side.backHidden = true;
        faces.addObj(side);
    }
    
    // creating the shaft
    lowerShaft = g.compilePath3D(["M",-width,width,0, "L",-width,3*width,0], "#686868", width/6);
    faces.addObj(lowerShaft);
      
    upperShaft = g.compilePath3D(["M",-width,3*width,0, "L",-26/15*width,13*width,0], "#A8A8A8", width/4);
    faces.addObj(upperShaft);
      
    grip = g.compilePath3D(["M",-26/15*width,13*width,0, "L",-2*width,17*width,0], "#202020", width/3);
    faces.addObj(grip);
      
      
    // add axes for visual debugging
    centerX = g.compilePath3D(["M",-4*width,0,0, "L",4*width,0,0], "green", 1);
    faces.addObj(centerX);
    centerY = g.compilePath3D(["M",0,-4*width,0, "L",0,4*width,0], "green", 1);
    faces.addObj(centerY);
    centerZ = g.compilePath3D(["M",0,0,-4*width, "L",0,0,4*width], "green", 1);
    faces.addObj(centerZ);
    
    
    return faces;
  }



  function buildCube(g, width, colors) { // pass width and array of 6 colors

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


  function applyZoffset(displacementframes){ 
    // applies the z offset to the displacement frames from the sensor
    //   -> returns the new displacement frames

      console.log("applyZoffset");
      newframes = [[0,0,0]];
    // Offset for displacement in the Z direction (can take decimal numbers!)
      Zoffset = [.5,.5,-1];

    // iterate through old frames
      var j = 1;  
      for (j = 1; j< displacementframes.length - 1; j++){
        
        // Calculate position increment from displacement data
        xdisp = displacementframes[j][0] - displacementframes[j-1][0];
        ydisp = displacementframes[j][1] - displacementframes[j-1][1];
        zdisp = displacementframes[j][2] - displacementframes[j-1][2];
        
        // take into account z bias and position increments
        xincrementer = xdisp + Zoffset[0]*(zdisp);
        yincrementer = ydisp + Zoffset[1]*(zdisp);
        zincrementer = Zoffset[2]*(zdisp);

        // update displacement frames based on z bias
        newframesX = newframes[j-1][0] + xincrementer;
        newframesY = newframes[j-1][1] + yincrementer;
        newframesZ = newframes[j-1][2] + zincrementer;
        
        // append new array
        newframes.push([newframesX,newframesY,newframesZ]);    
      }
      return newframes;
  }



  function putterDemo(scrnID)
  {
    var g = new Cango3D(scrnID),
      newPos = {x:0, y:0, z:0},  // avoid creating Objects in event handlers
      taggedFace, cube1,
      width = 20,
      colors0 = ["#686868","#686868","#686868","#686868","#686868","#686868", "red"],
      colors1 = ["red","orange","yellow","green","blue","purple"],
      iter = 0,
      coordsX = -$("#puttCanvas").attr("width")/2, // bottom leftmost pixel x location
      coordsY = -$("#puttCanvas").attr("height")/2, // bottom leftmose pixel y location
      xyspan = -coordsX*2,
      incCoords = 0;
      console.log(g.cnvs.offsetWidth);

    // Initialize subfunction within putterDemo()
      function movePutter() {
        // use target's parent group drawing origin as reference

        function nextframe (convertedframes){
        // note that z graphics are in towards the screen, thus Z and Y are switched
            X = convertedframes[iter][0];
            Y = convertedframes[iter][1];
            Z = convertedframes[iter][2];
            g.setWorldCoords3D(coordsX+X, coordsY+Z, xyspan+Y);
        }

        var useDataMode = 1; // set this to one to use parse data

        if (useDataMode){   // USE data mode
            if (iter == 0){
                cube1.transform.rotate(1,0,0,gyroData[iter][0]);
                cube1.transform.rotate(0,1,0,gyroData[iter][1]);
                //cube1.transform.rotate(0,0,1,gyroData[iter][2]);
                nextframe(displacementData); // sets world coordinates based on frame

            } else if (iter == gyroData.length - 1) {
                iter = -1; // reset the array counter for loop
                cube1.transform.reset(); // reset the 3d object

            } else {
            // subtract here because gyroData comes in as angular displacement, Cango accumulates
                cube1.transform.rotate(1,0,0,-(gyroData[iter][0] - gyroData[iter-1][0]));
                cube1.transform.rotate(0,1,0,-(gyroData[iter][1] - gyroData[iter-1][1]));
                //cube1.transform.rotate(0,0,1,-(gyroData[iter][2] - gyroData[iter-1][2]));
                nextframe(displacementData); // sets world coordinates based on frame
            }

            iter = iter + 1; // increment the array counter

        } else{ // not in USE data mode
         cube1.transform.rotate(1,1,0,1);
        }

        g.renderFrame(cube1); // update the canvas

      }


      g.setWorldCoords3D(coordsX, coordsY, xyspan);
      g.setFOV(45);
      g.setPropertyDefault("backgroundColor", "lightyellow");


      //cube1 = buildCube(g, width, colors1);
      cube1 = buildPutter(g, 10, colors0); // note: len(colors1) != len(colors0)

      displacementData = applyZoffset(displacementData); // data processing routine for graphics
  

      setInterval(movePutter, 25); // in msec

  }



    // return from the outermost function, myapp
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
jQuery(myapp.init); // render the outermost function



