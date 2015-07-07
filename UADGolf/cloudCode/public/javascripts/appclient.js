var myapp = (function(){

    var removeUsers = function() {
        console.log("inside remove users");
        var users = [];
        $("#remove-table tr.selected td.email").each(function() {
            users.push($(this).text());
        });
       
        $.post("/remove-user", {users:users}, function(data) {
            console.log("post is done");
            alert(data);
            window.location = "/remove-user";
            return;
        });
    }

   

    return {
        init: function() {
            //console.log("Client-side app starting up")
            
			//jQuery("#testbutton").click(test);
            
        }
    }
})();
jQuery(myapp.init);

