//Parse.initialize("iAFEw9XwderX692l0DGIwoDDHcLTGMGtcBFbgMqb", "d4q6T1YxleQ2za17PdTtfaAD8x6tAB9rdW4y9vMD");

exports.index = function (req, res) {
    var frames = [[1,2,3]];
    var putt = Parse.Object.extend("Putt");
    var query = new Parse.Query(putt);
    query.get("12fz4AHTDK", {
        success: function(data) {
            var frames = data.get("frames");   
            console.log("got frames");
            console.log(frames);   
            res.render('index', {title: "UADGolf", users:[], home:true, frames:frames});      
        },
        error: function(object, error) {
            console.log("Error getting putt data");
            console.log(error);
            frames = [[3,2,1]];
            res.render('index', {title: "UADGolf", users:[], home:true, frames:frames});
        }
    });
    
};

exports.documentation = function(req, res) {
    res.render('documentation', {title: "UADGolf", home:true, users:[]});
};

