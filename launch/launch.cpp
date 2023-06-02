/**
 * \file launch.cpp
 * \brief XML launch file for launching nodes.
 */

/**
 * \brief The main launch file for launching nodes.
 */
<?xml version="1.0"?>
<launch>
   <!-- <node pkg ="control" type="client_node" name="cilent" output="screen" /> -->

   /**
    * \brief Launches the client_sub_node.
    *
    * This node is responsible for <add description here>.
    */
   <node pkg ="control" type="client_sub_node" name="client_sub" output="screen" />

   /**
    * \brief Launches the info_counter_node.
    *
    * This node is responsible for <add description here>.
    */
   <node pkg ="control" type="info_counter_node" name="info_counter" output="screen" />

   /**
    * \brief Launches the info_sub_node.
    *
    * This node is responsible for <add description here>.
    */
   <node pkg ="control" type="info_sub_node" name="info_sub" output="screen" />
</launch>

