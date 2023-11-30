# base image
FROM node:16-alpine

# create our working directory
RUN mkdir -p /opt/

# copy data
COPY rotem_sela_frontend /opt/frontend

# set working directory
WORKDIR /opt/frontend

# add `/app/node_modules/.bin` to $PATH
ENV PATH /opt/frontend/node_modules/.bin:$PATH

# install and cache app dependencies
RUN npm install
RUN npm install @vue/cli@3.7.0 -g

# production build 
RUN npm run build

# production environment
FROM nginx:1.16.0-alpine
COPY --from=0 /opt/frontend/dist /usr/share/nginx/html
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
