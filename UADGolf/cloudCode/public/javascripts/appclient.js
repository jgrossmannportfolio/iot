var myapp = (function(){
    var displacementData = []; // preallocation for data reception from parse
    var gyroData = [];
    var animated = false;
    var canvasID = '';          // to be loaded with the canvas ID given by the corresponding page

    var delta_t = 12.5;                   // frame rate, in msec
    var worldScalingFactor = 0;         // canvas world scaling factor, will be set in setCanvasDim() 
    var pixelsPerMeter = 3779.527559055;// conversion
    var canvasPixelsPerMeter = 0;       // will be set in setCanvasDim()


    function setCanvasDim(canvasID) {
        width = window.innerWidth;
        height = window.innerHeight;
        $("#"+canvasID).attr("width", width);
        $("#"+canvasID).attr("height", height / 2.0);
        console.log("width: "+$("#"+canvasID).attr("width"));
        console.log("height: "+$("#"+canvasID).attr("height"));
        worldScalingFactor = (height/2)/(25*pixelsPerMeter);
        console.log(worldScalingFactor);
        canvasPixelsPerMeter = worldScalingFactor*pixelsPerMeter;
        console.log("set canvas dimensions");
    };

    function getPuttData() {
        if($("#puttCanvas").length) {
            canvasID = 'puttCanvas';
            console.log("found canvas");
            setCanvasDim(canvasID);
            
            console.log("getting put data");
            var putt = Parse.Object.extend("Putt");
            var query = new Parse.Query(putt);
            query.get("12fz4AHTDK", {
                success: function(data) {
                    
                        // receive data
                        displacementData = data.get("frames");
                        gyroData = data.get("gyro");
                    
                        displacementData = parseIncomingArray(displacementData);
                        //displacementData = changeSigns(displacementData);
                        gyroData = parseIncomingArray(gyroData);

                        // Debug tool - check that the lengths are valid in the console
                        console.log("received data (gyro, displacement)");
                        console.log(gyroData);
                        console.log(displacementData);
                        
                        // data processing routine for translation graphics
                        displacementData = applyZoffset(displacementData); 

                        // converting data to pixels
                        displacementData = toPixels(displacementData,canvasPixelsPerMeter);   
                        
                        console.log("processed data (gyro, displacement)");
                        console.log(gyroData);
                        console.log(displacementData);
                    
                    // jump to animation once the data has been received and processed
                    putterDemo(canvasID,displacementData,gyroData);   


                },
                error: function(object, error) {
                    console.log("Error getting putt data");
                    console.log(error);

                }

            });
        
        }
        // if SAMPLE PUTT ANIMATION page was rendered
        if($("#samplePuttCanvas").length) {
            
            canvasID = 'samplePuttCanvas';
            console.log("found canvas: "+canvasID);
            setCanvasDim(canvasID);
  
                    console.log("generateRandPath()");
                    var randIter = 1,
                        displacementData = [[0,0,0]],
                        gyroData = [[0,0,0]],
                        xPositionincrementer = 1,
                        yPositionincrementer = 0,
                        zPositionincrementer = 0;
                      
                    for (randIter; randIter < 300; randIter = randIter + 1 ){
                        xprev = displacementData[randIter-1][0];
                        yprev = displacementData[randIter-1][1];
                        zprev = displacementData[randIter-1][2];
                        displacementData.push([xprev-xPositionincrementer,yprev-yPositionincrementer,zprev-zPositionincrementer]);
                        gyroData.push([0,0,0]);
                    }
            displacementData = applyZoffset(displacementData);
                    console.log(displacementData);
                    console.log(gyroData);
                    putterDemo(canvasID,displacementData,gyroData);  
                



        }
        


       // ======== functions for data z ============
        function changeSigns (incomingArray){
            jj = 0;
            for (jj; jj < incomingArray.length; jj++){
                incomingArray[jj][1] = (-1)*incomingArray[jj][1];
            }
            return incomingArray;
        }

        
        function parseIncomingArray (incomingArray){
            if (incomingArray[0].length != 3){
                // append the incorrect parsings as the first array in the array
                console.log("python bug accounted for");   
                firstArray = [[incomingArray[0],incomingArray[1],incomingArray[2]]];
                incomingArray = incomingArray.slice(3, incomingArray.length);
                incomingArray = firstArray.concat(incomingArray);
            }
            return incomingArray;
        }


        function applyZoffset(displacementframes){ 
        // applies the z offset to the displacement frames from the sensor
        //   -> returns the new displacement frames
          //var displacementframes = displacementframes;
          console.log("applyZoffset()");
          newframes = [[0,0,0]];

            //Create Z-offset matrix via screen size
            pageWidth = $("#"+canvasID).attr("width")-0; //subtraction converts str to int
            pageHeight = $("#"+canvasID).attr("height")-0;
            if (pageWidth > pageHeight ){
                z1 = .5;
                z2 = pageHeight/(2*pageWidth);
            } else {
                z1 = pageWidth/(pageWidth+pageHeight);
                z2 = pageHeight/(pageWidth+pageHeight);
            }
            z3 = -1;
            Zoffset = [z1,z2,z3]; 

        // iterate through old frames
          var j = 1;  
          for (j = 1; j< displacementframes.length; j++){


            // Calculate position increment from displacement data
            xdisp = displacementframes[j][0] - displacementframes[j-1][0];
            ydisp = displacementframes[j][1] - displacementframes[j-1][1];
            zdisp = displacementframes[j][2] - displacementframes[j-1][2];
            
            // take into account z bias and position increments
            yincrementer = ydisp + Zoffset[0]*(xdisp);
            zincrementer = zdisp + Zoffset[1]*(xdisp);
            xincrementer = Zoffset[2]*(xdisp);
        
            // update displacement frames based on z bias
            newframesY = newframes[j-1][1] + yincrementer;
            newframesZ = newframes[j-1][2] + zincrementer;
            newframesX = newframes[j-1][0] + xincrementer;
            
            // append new array
            newframes.push([newframesX,newframesY,newframesZ]);    
          }

          return newframes;
       } // ends applyZoffset()


      function toPixels(displacementframes,scale){
            console.log("toPixels()");
            var ii = 0,
                newframes = [];        
            for (ii; ii < displacementframes.length; ii++){
                pixX = scale*displacementframes[ii][0];
                pixY = scale*displacementframes[ii][1];
                pixZ = scale*displacementframes[ii][2];
                newframes.push([pixX,pixY,pixZ]);
            }

            return newframes;
       }
        
    }; // ends getPuttData()


            


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
      var canvas = document.getElementById(canvasID);
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



// ----------------------------------------------------


  function buildPutter(g, height, colors){
    console.log("buildingputter()");
    // height input in pixels scaled to the canvas height
    // Customize your putter
    buttDisplacementScalar = 6; // essentially the x displacement from tip of the shaft to the putter head
    putterHeightScalar = 17;
    width = height/putterHeightScalar;
    
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

    for (i=0; i<7; i++){
        //console.log(i)
        side = g.compileShape3D(sq[i], colors[i]);
        side.translate(translateX[i], translateY[i], translateZ[i]);
        side.rotate(rotateX[i], rotateY[i], rotateZ[i], rotateMag[i]);
        //side.backHidden = true;
        faces.addObj(side);
    }
    
    shaft2ShaftPlusGripRatio = 7/10;
    // creating the shaft
    lowerShaft = g.compilePath3D(["M",-width,width,0, "L",-width,2*width,0], "#686868", width/3);
    faces.addObj(lowerShaft);
      
    upperShaft = g.compilePath3D(["M",-width,2*width,0, "L",(shaft2ShaftPlusGripRatio)*(-buttDisplacementScalar*width),(shaft2ShaftPlusGripRatio)*(putterHeightScalar*width),0], "#A8A8A8", width/4);
    faces.addObj(upperShaft);
      
    grip = g.compilePath3D(["M",(shaft2ShaftPlusGripRatio)*(-buttDisplacementScalar*width),(shaft2ShaftPlusGripRatio)*(putterHeightScalar*width),0, "L",(-buttDisplacementScalar*width),(putterHeightScalar*width),0], "#202020", width/3);
    faces.addObj(grip);
      
    // reference putter head to the center
    faces.translate(0,-width/2,0);
      
    // add axes for visual debugging
    centerX = g.compilePath3D(["M",-4*width,0,0, "L",4*width,0,0], "green", 1);
    faces.addObj(centerX);
    centerY = g.compilePath3D(["M",0,-4*width,0, "L",0,4*width,0], "green", 1);
    faces.addObj(centerY);
    centerZ = g.compilePath3D(["M",0,0,-4*width, "L",0,0,4*width], "green", 1);
    faces.addObj(centerZ);

    
    return faces;
  }





// ============================ MOVE PUTTER ==================================
  function putterDemo(scrnID,displacementData,gyroData)
  {
    var g = new Cango3D(scrnID),
      newPos = {x:0, y:0, z:0},  // avoid creating Objects in event handlers
      taggedFace, cube1,
      width = 20,
      colors = ["#686868","#686868","#686868","#686868","#686868","#686868", "red"],
      iter = 0,
      timer = 0,
      coordsX = -($("#"+canvasID).attr("width")/2), // bottom leftmost pixel x location
      coordsY = -($("#"+canvasID).attr("height")/2), // bottom leftmose pixel y location
      xyspan = -(2*coordsX);
      console.log(g.cnvs.offsetWidth);

    // Initialize subfunction within putterDemo()
      function movePutter() {
            //console.log(iter);
        
            function nextframe (convertedframes){
            // note that z graphics are in towards the screen, thus Z and Y are switched
                X = convertedframes[iter][1];
                Y = convertedframes[iter][2];
                Z = convertedframes[iter][0];
                console.log("xyz:");
                console.log(X);
                console.log(Y);
                console.log(Z);

                // ========== Y DIRECTION TEMPORARILY EXCLUDED DUE TO GRAVITY DEBUG
                //Y = 0;


                g.setWorldCoords3D(coordsX-X, coordsY-Y, xyspan-Z);
            }

            var useDataMode = 1; // set this to one to use parse data
            
            if (useDataMode){   // USEdata mode
                if (iter == 0){
                    //console.log("iter == 0");
                    //console.log(gyroData[iter][0]);
                    //console.log("gyro");
                    //console.log(gyroData);
                    //console.log("displacement");
                    //console.log(displacementData);
                    cube1.transform.rotate(0,0,1,gyroData[iter][0]);
                    cube1.transform.rotate(1,0,0,gyroData[iter][1]);
                    cube1.transform.rotate(0,1,0,gyroData[iter][2]);
                    nextframe(displacementData); // sets world coordinates based on frame

                } else if (iter == gyroData.length - 1) {
                    console.log("LOOP");
                    iter = -1; // reset the array counter for loop
                    //cube1.transform.reset(); // reset the 3d object
                    
                    clearTimeout(timer);
                    
                    setTimeout(function() {
                        cube1.transform.reset(); // reset the 3d object
                        timer = setInterval(movePutter, delta_t);
                    }, 1800);

                } else {
                console.log("displacement");
                // subtract here because gyroData comes in as angular displacement, Cango accumulates
                    cube1.transform.rotate(0,0,1,-(gyroData[iter][0] - gyroData[iter-1][0]));
                    cube1.transform.rotate(1,0,0,-(gyroData[iter][1] - gyroData[iter-1][1]));
                    cube1.transform.rotate(0,1,0,-(gyroData[iter][2] - gyroData[iter-1][2]));
                    nextframe(displacementData); // sets world coordinates based on frame
                }

                iter = iter + 1; // increment the array counter

            } else{ // not in USEdata mode
             cube1.transform.rotate(1,0,0,10);
            }

            g.renderFrame(cube1); // update the canvas
      } // ends movePutter()


      g.setWorldCoords3D(coordsX, coordsY, xyspan);
      g.setFOV(45);
      g.setPropertyDefault("backgroundColor", "lightyellow");
    
      // build putter object
      putterHeight = .83;  // in meters
      cube1 = buildPutter(g, canvasPixelsPerMeter*(putterHeight*10), colors); // note for builfing cube/putter: len(colors1) != len(colors0)
      console.log("Putter Built");
      timer = setInterval(movePutter, delta_t); // in msec
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


