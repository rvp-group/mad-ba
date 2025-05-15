#!/usr/bin/python3
import json
import argparse
import sys, os

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def pushImages():
    print(bcolors.OKBLUE + "pushing plots to wiki" +  bcolors.ENDC)
    plot_ape_file = os.environ['PLOT_APE']
    plot_rpe_file = os.environ['PLOT_RPE']
    print("plots files:" +  bcolors.OKBLUE + plot_ape_file +  bcolors.ENDC + " " +  bcolors.OKBLUE + plot_rpe_file + bcolors.ENDC)
    if (len(plot_ape_file) == 0 or len(plot_rpe_file) == 0):
        print(bcolors.FAIL + "either ape or rpe plots do not exist"+  bcolors.ENDC)
        sys.exit(-2)
    token = os.environ['ARTIFACT_PRIVATE_TOKEN']
    project_name = os.environ['CI_PROJECT_NAME']
    if (len(token) == 0 or len(project_name) == 0):
        print(bcolors.FAIL + "either token or project_name do not exist"+  bcolors.ENDC)
        sys.exit(-2)
    url_base = "https://gitlab.com/api/v4/projects/srrg-software%2F" + project_name
    url_wikis = url_base + "/wikis"
    command = "curl -s --header \"PRIVATE-TOKEN: " + token + "\" " + url_wikis
    wikis_res_json=os.popen(command).read()
    # print(wikis_res_json)
    wikis_res = json.loads(wikis_res_json)
    if ('message' in wikis_res):
        print(bcolors.FAIL + wikis_res['message'] +  bcolors.ENDC)
        return

    from datetime import datetime
    new_commit_datetime = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    # print(new_commit_datetime)
    for page in wikis_res:
        if page['title'] != 'Results':
            continue
        slug = "/" + page['slug']
        url_page = url_wikis + slug
        command = "curl -s --header \"PRIVATE-TOKEN: " + token + "\" " + url_page
        page_res_json=os.popen(command).read()
        # print(page_res_json)
        page_res = json.loads(page_res_json)
        if not ('content' in page_res):
            print(bcolors.FAIL + "no content found" +  bcolors.ENDC)
            return
        # print(page_res['content'])

        url_upload = url_base + '/wikis/attachments'
        command = "curl --request POST --header \"PRIVATE-TOKEN: " + token + "\"  --form \"file=@" + plot_rpe_file + "\" " + url_upload
        upload_image_res_json=os.popen(command).read()
        upload_image_res = json.loads(upload_image_res_json)

        # print(upload_image_res_json)
        markdown = ""
        if ('link' in upload_image_res):
            print(bcolors.OKBLUE + upload_image_res['link']['markdown'] +  bcolors.ENDC)
            markdown = upload_image_res['link']['markdown']

        if (len(markdown) == 0):
            return

        new_result_content = "## " + new_commit_datetime + "\n" + markdown
        new_content = new_result_content + "\n" + page_res['content']
        print(new_content)
        uploading_url = url_wikis + "/" + page['title']
        data_arg = "--data \"content="+new_content+"\""
        command = "curl --request PUT "+data_arg+" --header \"PRIVATE-TOKEN: " + token + "\" "+uploading_url
        uploading_json=os.popen(command).read()
        print(uploading_json)

parser = argparse.ArgumentParser(description='''
    Read rmse from file and evalue if benchmark passed or not.
    ''')
parser.add_argument('file', help='json result file from evo-suite')
parser.add_argument('--threshold', help='threshold for benchmark passed',default=0.5)

args = parser.parse_args()

filename = args.file;
threshold = args.threshold;

print("Reading file " + bcolors.OKBLUE + filename +  bcolors.ENDC)
print("Threshold is set to " + bcolors.OKBLUE + threshold +  bcolors.ENDC)

with open(filename, 'r') as f:
  results = json.load(f)
  rmse = results['rmse']
  print("RMSE value: "+ bcolors.WARNING+ bcolors.UNDERLINE + str(rmse) +  bcolors.ENDC)
  if (float(rmse) > float(threshold)):
    print("RMSE above the threshold, benchmark "+ bcolors.FAIL +"NOT passed"+  bcolors.ENDC);
    sys.exit(-1)
  else:
    print("RMSE below the threshold, benchmark "+ bcolors.OKGREEN +"PASSED"+  bcolors.ENDC);
    #pushImages()
    sys.exit(0)
