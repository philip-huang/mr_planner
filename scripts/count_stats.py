import pandas as pd

# Read the CSV data into a DataFrame

# Define a function to calculate the average improvement
def average_improvement(df, out_df=None):
    # Add percentage improvement columns to the DataFrame
    df['flowtime_improvement_perc'] = ((df['flowtime_pre'] - df['flowtime_post']) / df['flowtime_pre']) * 100
    df['makespan_improvement_perc'] = ((df['makespan_pre'] - df['makespan_post']) / df['makespan_pre']) * 100

    # Calculate the average percentage improvements
    avg_flowtime_improvement_perc = df['flowtime_improvement_perc'].mean()
    avg_makespan_improvement_perc = df['makespan_improvement_perc'].mean()

    # Append a row with the averages
    df.loc['Average'] = df.mean()

    if out_df:
        df.to_csv(out_df, sep=',')
    
    return avg_flowtime_improvement_perc, avg_makespan_improvement_perc

# Calculate the average improvements
df = pd.read_csv("../outputs/dual_gp4_benchmark.csv", sep=', ')

print(average_improvement(df, out_df="../outputs/dual_gp4_benchmark_avg.csv"))