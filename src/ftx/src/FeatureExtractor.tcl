namespace eval ftx {

sta::define_cmd_args "init" {}
proc init {} {
  init_cmd
}

sta::define_cmd_args "init_graph" {}
proc init_graph {} {
  init_graph_cmd
}

sta::define_cmd_args "init_graph_from_GCells" {}
proc init_graph_from_GCells {} {
  init_graph_from_GCells_cmd
}

sta::define_cmd_args "read_congestion" { file_name }
proc read_congestion { arg } {
  set file_name $arg
  read_congestion_cmd $file_name
}

sta::define_cmd_args "run" {}
proc run {} {
  run_cmd
}

sta::define_cmd_args "write_csv" { file_name }
proc write_csv { arg } {
  set file_name $arg
  write_csv_cmd $file_name
}

sta::define_cmd_args "read_rpt" { file_name }
proc read_rpt { arg } {
  set file_name $arg
  read_rpt_cmd $file_name 0
}

sta::define_cmd_args "read_rpt_triton" { file_name }
proc read_rpt_triton { arg } {
  set file_name $arg
  read_rpt_cmd $file_name 1
}

sta::define_cmd_args "calculate_ABU" {}
proc calculate_ABU {} {
  calculate_ABU_cmd
}

sta::define_cmd_args "Draw_DRVs" {}
proc Draw_DRVs {} {
  Draw_DRVs_cmd
}

sta::define_cmd_args "Draw_Grid" {}
proc Draw_Grid {} {
  Draw_Grid_cmd
}

sta::define_cmd_args "Paint_Nodes" { file_name }
proc Paint_Nodes { arg } {
  set file_name $arg
  Paint_Nodes_cmd $file_name
}

# ftx namespace end
}
