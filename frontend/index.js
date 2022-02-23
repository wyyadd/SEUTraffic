var simulation = replay_roadnet
var viewport, simulatorContainer;
var roadnet = simulation.static

console.log(roadnet)

let Application = PIXI.Application,
Sprite = PIXI.Sprite,
Graphics = PIXI.Graphics,
Container = PIXI.Container,
ParticleContainer = PIXI.ParticleContainer,
Texture = PIXI.Texture,
Rectangle = PIXI.Rectangle;

//创建pixi app
let app = new PIXI.Application({
width: 512,
height: 512,
transparent: false,
backgroundColor: 0xe8ebed
});
renderer = app.renderer;
//Add the canvas that Pixi automatically created for you to the HTML document
document.body.appendChild(app.view);
            
//创建viewport

viewport = new pixi_viewport.Viewport({

    screenWidth: window.innerWidth,
    screenHeight: window.innerHeight,
    worldWidth: 1000,
    worldHeight: 1000,
    interaction: app.renderer.plugins.interaction
});

app.stage.addChild(viewport);

viewport
    .drag()//拖拽
    .pinch()
    .wheel()//滚轮放大
    .decelerate();//减速
            
//添加一个方框

const sprite = viewport.addChild(new PIXI.Sprite(PIXI.Texture.WHITE))
sprite.tint = 0xff0000
sprite.width = sprite.height = 100
sprite.position.set(100, 100)

            /*===pixi-viewport 使用样例 
            const app = new PIXI.Application()
            document.body.appendChild(app.view)

            // create viewport
            const viewport = new pixi_viewport.Viewport({
                screenWidth: window.innerWidth,
                screenHeight: window.innerHeight,
                worldWidth: 1000,
                worldHeight: 1000,
                interaction: app.renderer.plugins.interaction // the interaction module is important for wheel to work properly when renderer.view is placed or scaled
            })

            // add the viewport to the stage
            app.stage.addChild(viewport)

            // activate plugins
            viewport
                .drag()
                .pinch()
                .wheel()
                .decelerate()

            // add a red box
            const sprite = viewport.addChild(new PIXI.Sprite(PIXI.Texture.WHITE))
            sprite.tint = 0xff0000
            sprite.width = sprite.height = 100
            sprite.position.set(100, 100) */

function drawNode(node, graphics) {
    graphics.beginFill(0x586970);//填充画笔颜色 
    let outline = node.outline;
    for (let i = 0 ; i < outline.length ; i+=2) { 
        outline[i+1] = -outline[i+1];
        if (i == 0)
            graphics.moveTo(outline[i], outline[i+1]);
        else
            graphics.lineTo(outline[i], outline[i+1]);
    }
    graphics.endFill();
            
    if (debugMode) {
        graphics.hitArea = new PIXI.Polygon(outline);
        graphics.interactive = true;
        graphics.on("mouseover", function () {
            selectedDOM.innerText = node.id;
            graphics.alpha = 0.5;
        });
        graphics.on("mouseout", function () {
            graphics.alpha = 1;
        });
    }
            
}