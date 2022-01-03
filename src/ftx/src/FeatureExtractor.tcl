namespace eval ftx {

sta::define_cmd_args "feature_extract_init" {}
proc feature_extract_init {} {
  feature_extract_init_cmd
}

sta::define_cmd_args "feature_extract_init_graph" {}
proc feature_extract_init_graph {} {
  feature_extract_init_graph_cmd
}

sta::define_cmd_args "feature_extract_init_graph_from_GCells" {}
proc feature_extract_init_graph_from_GCells {} {
  feature_extract_init_graph_from_GCells_cmd
}

sta::define_cmd_args "feature_extract_read_congestion" { file_name }
proc feature_extract_read_congestion { arg } {
  set file_name $arg
  feature_extract_read_congestion_cmd $file_name
}

sta::define_cmd_args "feature_extract_run" {}
proc feature_extract_run {} {
  feature_extract_run_cmd
}

sta::define_cmd_args "feature_extract_write_csv" { file_name }
proc feature_extract_write_csv { arg } {
  set file_name $arg
  feature_extract_write_csv_cmd $file_name
}

sta::define_cmd_args "feature_extract_read_rpt" { file_name }
proc feature_extract_read_rpt { arg } {
  set file_name $arg
  feature_extract_read_rpt_cmd $file_name 0
}

sta::define_cmd_args "feature_extract_read_rpt_triton" { file_name }
proc feature_extract_read_rpt_triton { arg } {
  set file_name $arg
  feature_extract_read_rpt_cmd $file_name 1
}

sta::define_cmd_args "feature_extract_calculate_ABU" {}
proc feature_extract_calculate_ABU {} {
  feature_extract_calculate_ABU_cmd
}

sta::define_cmd_args "feature_extract_Draw_DRVs" {}
proc feature_extract_Draw_DRVs {} {
  feature_extract_Draw_DRVs_cmd
}

sta::define_cmd_args "feature_extract_Paint_Nodes" { file_name }
proc feature_extract_Paint_Nodes { arg } {
  set file_name $arg
  feature_extract_Paint_Nodes_cmd $file_name
}

# ftx namespace end
}
