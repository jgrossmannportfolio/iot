

exports.index = function (req, res) {
    console.log("rendering index");
    res.render('index', {title: "UADGolf", users:[], home:true});
};

