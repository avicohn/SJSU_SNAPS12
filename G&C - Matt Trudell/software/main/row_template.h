/* row_template.h
 *
 * Author: Matthew Trudell
 * Created: 4-21-13
 * Description: Defines format of columns of Trajectory Matrix (template_matrix)
 * Notes: See 2012-2013 G&C report for reference.
 */

#ifndef ROW_TEMPLATE_H_
#define ROW_TEMPLATE_H_

struct row_template_matrix
{
	int t;
	float d_exp;
	float v_exp;
	float v_para_range_low;
	float v_para_range_high;
	float v_perp_max;
	float thrust_heading_des;
	float impulse_des;
};

#endif /* ROW_TEMPLATE_H_ */
