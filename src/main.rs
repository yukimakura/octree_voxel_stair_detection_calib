use std::fs;
use std::io::{BufWriter, Write};

use std::sync::{Mutex, Arc};

mod v_msg {
    rosrust::rosmsg_include!(visualization_msgs/Marker);
}

mod f_msg {
    rosrust::rosmsg_include!(std_msgs/Float64);
}

struct PARAMETERS{
    low_point_delta_threshold : f64,
    low_point_delta_range : f64,
    x_axis_width_range : f64,
    detect_range : f64,
}

fn ave_calc(datas :&Vec<f64>) -> f64{
    let mut sum = 0.0;
    for item in datas{
        sum += *item as f64;
    }

    sum/(datas.len() as f64)
}

fn file_output(path : String , sigma2 : f64 , ave : f64) -> Result<(),Box<std::error::Error>>{

    let mut f = BufWriter::new(fs::File::create(path).unwrap());
    writeln!(f, "{}",1.0).unwrap();//dummy
    writeln!(f, "{}",sigma2).unwrap();
    write!(f, "{}",ave).unwrap();

    Ok(())
}

fn get_parameters(p : &mut PARAMETERS){
    p.low_point_delta_threshold = rosrust::param("~low_point_delta_threshold").unwrap().get::<f64>().unwrap_or(0.20);
    p.low_point_delta_range = rosrust::param("~low_point_delta_range").unwrap().get::<f64>().unwrap_or(0.10);
    p.x_axis_width_range = rosrust::param("~x_axis_width_range").unwrap().get::<f64>().unwrap_or(0.05);
    p.detect_range = rosrust::param("~detect_range").unwrap().get::<f64>().unwrap_or(1.0);
}

fn main() {

    // Initialize node
    rosrust::init("Marker_listener");
    let mut datas_vec : Vec<f64> = Vec::new();
    let mut params = PARAMETERS{low_point_delta_threshold : 0.20,low_point_delta_range : 0.10,x_axis_width_range : 0.05,detect_range : 1.0 };
    get_parameters(&mut params);
    // mutex vars
    let mut x0_pub_mutex = Arc::new(Mutex::new(rosrust::publish("/x0points", 100).unwrap()));
    let mut datas_vec_mutex = Arc::new(Mutex::new(datas_vec));
    let mut sub_cnt_mutex = Arc::new(Mutex::new(0i64));
    
    let marker_sub = rosrust::subscribe("/jsk/marker", 10, move |v: v_msg::visualization_msgs::Marker| {

        let mut x0_pub = x0_pub_mutex.lock().unwrap();
        // Callback for handling received messages
        let mut delta_z = f_msg::std_msgs::Float64::default();
        let mut x0_points : Vec<v_msg::geometry_msgs::Point> = Vec::new();
        let mut x0_pub_msgs = v.clone();
        let mut x0_colors : Vec<v_msg::std_msgs::ColorRGBA> = Vec::new();

        // rosrust::ros_info!("Received: {}, Color :{:?}", &v.points.len(),&v.colors.get(5));
        let mut min_y = v_msg::geometry_msgs::Point{x:0.0,y:0.0,z:100.0};
        let mut delta_y_point = v_msg::geometry_msgs::Point::default();
        for (cnt,item) in v.points.iter().enumerate(){
            if item.x.abs() <= params.x_axis_width_range {
                let red = v_msg::std_msgs::ColorRGBA{r:1.0,g:0.0,b:0.0,a:1.0,};
                x0_points.push(v_msg::geometry_msgs::Point::clone(item));
                // x0_colors.push(v_msg::std_msgs::ColorRGBA::clone(&v.colors[cnt]));
                x0_colors.push(red);
                if min_y.y > item.y{
                    min_y = v_msg::geometry_msgs::Point::clone(item);
                }
            }
        }
        for item in &x0_points{
            if (min_y.y + params.low_point_delta_threshold) < item.y  && item.y < (min_y.y + (params.low_point_delta_threshold+params.low_point_delta_range)){ //一番下のポイントから大体20〜30cmぐらい上のポイントを捜索
                delta_y_point = v_msg::geometry_msgs::Point::clone(item);
            }
        }
        // rosrust::ros_info!("min_y point: {:?},\tdelta_y_point: {:?},\t z delta: {}",min_y.y,delta_y_point.y,min_y.z-delta_y_point.z);
        rosrust::ros_info!("z info = (min_y point: {:?},\tdelta_y_point: {:?})",min_y.z,delta_y_point.z);
        
        x0_pub_msgs.type_ = 6;
        x0_pub_msgs.points = x0_points;
        x0_pub_msgs.colors = x0_colors;
        x0_pub_msgs.scale.x = 3.0;
        x0_pub_msgs.scale.y = 3.0;

        datas_vec_mutex.lock().unwrap().push(min_y.z - delta_y_point.z);
        let mut sub_cnt = sub_cnt_mutex.lock().unwrap();
        *sub_cnt += 1;
        x0_pub.send(x0_pub_msgs).unwrap();
        println!("Call back number = {}",sub_cnt);

        if *sub_cnt > 1000{
            let ave = ave_calc(&datas_vec_mutex.lock().unwrap());
            //S
            let mut S = 0.0;
            for item in datas_vec_mutex.lock().unwrap().iter(){
                S += (*item as f64 - ave).powf(2.0);
            }
            let sigma2 = S/(datas_vec_mutex.lock().unwrap().len() as f64);

            //file output

            println!("filename = plane_params.csv , sigma^2 = {}, average = {}",sigma2,ave);
            file_output("plane_params.csv".to_string(), sigma2 , ave);
            std::process::exit(0);
        }

    }).unwrap();

    rosrust::spin();
}