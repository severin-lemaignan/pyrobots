saved_res=[]

def saveans(rqst):
	saved_res = []	
	global saved_res
	
	ans = rqst[1]
	for element in ans[:]:
		saved_res.append(float(element))

def back_ans():
	return saved_res
